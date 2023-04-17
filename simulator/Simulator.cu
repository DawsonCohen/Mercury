#include "Simulator.h"
#include "sim_kernel.cu"
#include <math.h>
#include <algorithm>
#include <functional>
#include <fstream>
#include <iostream>
#include <random>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

Simulator::Simulator(Element prototype, uint maxElements) :
	massesPerElement(prototype.masses.size()),
	springsPerElement(prototype.springs.size()),
	maxElements(maxElements),
	maxMasses(prototype.masses.size()*maxElements),
	maxSprings(prototype.springs.size()*maxElements),
	maxEnvs(1),
	m_hPos(0),
	m_hVel(0)
{
	m_dPos[0] = m_dPos[1] = 0;
    m_dVel[0] = m_dVel[1] = 0;
	
	_initialize();
}

Simulator::~Simulator() {
	// Free CPU
	delete[] m_hPos;
	delete[] m_hVel;

	delete[] m_hLbars;
	delete[] m_hPairs;
	delete[] m_hMats;

	delete[] massBuf;
	delete[] springBuf;
	delete[] envBuf;
	delete[] offsetBuf;

	delete[] m_hMaxStressCount;
	delete[] m_hMinStressCount;

	// Free GPU
	cudaFree((void**) m_dPos[0]);
	cudaFree((void**) m_dPos[1]);
	cudaFree((void**) m_dVel[0]);
	cudaFree((void**) m_dVel[1]);

	cudaFree((void**) m_dLbars);
	cudaFree((void**) m_dPairs);
	cudaFree((void**) m_dMats);

	cudaFree((void**) m_dMaxStressCount);
	cudaFree((void**) m_dMinStressCount);
	cudaFree((void**) m_dStresses);
	cudaFree((void**) m_dSpringIDs);

	cudaFree((void**) m_dMatEncodings);
}

void Simulator::Initialize(Element prototype, uint maxElements) {
	
	massesPerElement = prototype.masses.size();
	springsPerElement = prototype.springs.size();
	this->maxElements = maxElements;
	maxMasses = prototype.masses.size()*maxElements;
	maxSprings = prototype.springs.size()*maxElements;
	maxEnvs = 1;
	m_currentRead = 0;
	m_currentWrite = 1;

	_initialize();
}

void Simulator::_initialize() { //uint maxMasses, uint maxSprings) {
	if(initialized) {
		// Free CPU
		delete[] m_hPos;
		delete[] m_hVel;

		delete[] m_hLbars;
		delete[] m_hPairs;
		delete[] m_hMats;

		delete[] massBuf;
		delete[] springBuf;
		delete[] envBuf;
		delete[] offsetBuf;

		delete[] m_hMaxStressCount;
		delete[] m_hMinStressCount;

		delete[] m_hMatEncodings;

		// Free GPU
		cudaFree((void**) m_dPos[0]);
		cudaFree((void**) m_dPos[1]);
		cudaFree((void**) m_dVel[0]);
		cudaFree((void**) m_dVel[1]);

		cudaFree((void**) m_dLbars);
		cudaFree((void**) m_dPairs);
		cudaFree((void**) m_dMats);

		cudaFree((void**) m_dMaxStressCount);
		cudaFree((void**) m_dMinStressCount);
		cudaFree((void**) m_dStresses);
		cudaFree((void**) m_dSpringIDs);

		cudaFree((void**) m_dMatEncodings);
	}
	initialized = true;
	
	printf("Num Masses:\t%u\n",maxMasses);
	printf("Num Springs:\t%u\n",maxSprings);
	
	massBuf   = new Mass[maxMasses];
	springBuf = new Spring[maxSprings];
	offsetBuf = new uint[maxSprings];
	envBuf 	  =	new Environment[1];

	m_hLbars  = new float[maxSprings];
	m_hPairs  = new ushort[maxSprings*2];
	m_hMats   = new float[maxSprings*4];

	m_hPos 	  = new float[maxMasses*4];
    m_hVel 	  = new float[maxMasses*4];

	m_hMaxStressCount = new ushort[maxSprings];
	m_hMinStressCount  = new ushort[maxSprings];

	m_hStresses  = new float[maxSprings];
	m_hSpringIDs = new uint[maxSprings];

	m_hMatEncodings = new char[maxSprings];
	
	memset(m_hPos, 0, maxMasses*4*sizeof(float));
    memset(m_hVel, 0, maxMasses*4*sizeof(float));

	memset(m_hMaxStressCount, 0, maxSprings*sizeof(ushort));
    memset(m_hMinStressCount, 0, maxSprings*sizeof(ushort));

    memset(m_hStresses, 0, maxSprings * sizeof(float));
    memset(m_hSpringIDs, 0, maxSprings * sizeof(uint));

	memset(m_hMatEncodings, 0, maxSprings * sizeof(char));
	
    unsigned int massSizefloat4     = sizeof(float)  * 4 * maxMasses;
    unsigned int springSizeushort	= sizeof(ushort) * 1 * maxSprings;
    unsigned int springSizefloat    = sizeof(float)  * 1 * maxSprings;
    unsigned int springSizefloat4   = sizeof(float)  * 4 * maxSprings;
    unsigned int springSizeushort2  = sizeof(ushort) * 2 * maxSprings;
    unsigned int springSizeuint     = sizeof(uint)   * 1 * maxSprings;
    unsigned int springSizechar     = sizeof(char)   * 1 * maxSprings;
	
	cudaMalloc((void**)&m_dVel[0], massSizefloat4);
	cudaMalloc((void**)&m_dVel[1], massSizefloat4);

	cudaMalloc((void**)&m_dPos[0], massSizefloat4);
	cudaMalloc((void**)&m_dPos[1], massSizefloat4);

	cudaMalloc((void**)&m_dPairs,  springSizeushort2);
	cudaMalloc((void**)&m_dLbars,  springSizefloat);
	cudaMalloc((void**)&m_dMats,   springSizefloat4);
	cudaMalloc((void**)&m_dMats,   springSizefloat4);
	
	cudaMalloc((void**)&m_dMaxStressCount, springSizeushort);
	cudaMalloc((void**)&m_dMinStressCount,  springSizeushort);

	cudaMalloc((void**)&m_dStresses,  springSizefloat);
	cudaMalloc((void**)&m_dSpringIDs,  springSizeuint);

	cudaMalloc((void**)&m_dMatEncodings,  springSizechar);

	envBuf[0] = Environment();
	envCount++;
}

std::vector<ElementTracker> Simulator::Simulate(std::vector<Element>& elements) {
	numMasses = 0; numSprings = 0; numElements = 0;

	std::vector<ElementTracker> trackers = Allocate(elements);
	
	float3 gravity = {envBuf[0].g.x(), envBuf[0].g.y(), envBuf[0].g.z()};
	float stiffness = envBuf[0].floor_stiffness;
	float mu = envBuf[0].friction;
	float zeta = envBuf[0].damping;
	float step_time = 0;
	Eigen::Vector3f pos, vel;
	for(uint i = 0; i < numMasses; i++) {
		float  mass = massBuf[i].mass;
		vel  = massBuf[i].vel;
		pos = massBuf[i].pos;

		m_hPos[4*i]   = pos.x();
		m_hPos[4*i+1] = pos.y();
		m_hPos[4*i+2] = pos.z();
		m_hPos[4*i+3] = mass;
		
		m_hVel[4*i]   = vel.x();
		m_hVel[4*i+1] = vel.y();
		m_hVel[4*i+2] = vel.z();
	}

	for(uint i = 0; i < numSprings; i++) {
		Material mat    = springBuf[i].material;
		float    lbar   = springBuf[i].mean_length;
		uint	 left   = springBuf[i].m0,
			     right  = springBuf[i].m1;

		// printf("%u:\t%u\n",offsetBuf[i],springBuf[i].m0);

		m_hMats[4*i]   = mat.k;
		m_hMats[4*i+1] = mat.dL0;
		m_hMats[4*i+2] = mat.omega;
		m_hMats[4*i+3] = mat.phi;

		m_hMatEncodings[i] = mat.encoding;

		m_hPairs[2*i]   = left;
		m_hPairs[2*i+1] = right;
		m_hLbars[i] 	= lbar;

		m_hSpringIDs[i] = i;
	}

	for(uint i = 0; i < ACTIVE_MATERIAL_COUNT; i++) {
		Material m = materials::matLookup(i);
		m_hMaterials[4*i]   = m.k;
		m_hMaterials[4*i+1] = m.dL0;
		m_hMaterials[4*i+2] = m.omega;
		m_hMaterials[4*i+3] = m.phi;
	}

	cudaMemcpy(m_dVel[m_currentRead], m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPos[m_currentRead], m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPairs,  m_hPairs,  numSprings *2*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dLbars,  m_hLbars,  numSprings  * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMats,   m_hMats,   numSprings *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMatEncodings,   m_hMatEncodings,   numSprings *sizeof(char), cudaMemcpyHostToDevice);
	#ifdef FULL_STRESS
	cudaMemcpy(m_dStresses,   m_hStresses,   numSprings*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringIDs,   m_hSpringIDs,   numSprings*sizeof(uint), cudaMemcpyHostToDevice);
	#endif

	cudaMemcpy(m_dMaxStressCount,   m_hMaxStressCount,   numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMinStressCount,   m_hMinStressCount,   numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dMatEncodings, m_hMatEncodings, numSprings*sizeof(char), cudaMemcpyHostToDevice);
	cudaMemcpyToSymbol(d_materials, m_hMaterials, 4*4*sizeof(float));

	gpuErrchk( cudaPeekAtLastError() );
	
	/*
	Notes on SM resources:
	thread blocks	: 8
	threads			: 2048
	registers		: 65536
	shared mem		: 49152
	*/

	uint maxSharedMemSize = 49152;
	uint bytesPerMass = sizeof(float3) + sizeof(float3);
	uint bytesPerElement = massesPerElement*bytesPerMass;
	// Must equal 1 for proper max/min spring calculation
	uint elementsPerBlock = min(maxSharedMemSize / bytesPerElement, numElements);
	uint massesPerBlock = massesPerElement * elementsPerBlock;
	uint springsPerBlock = springsPerElement * elementsPerBlock;
	uint sharedMemSize = massesPerBlock * bytesPerMass;
	int numBlocks = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	assert(sharedMemSize < maxSharedMemSize);
	// uint bytesPerBlock = elementsPerBlock * bytesPerElement;
	// int numBlocks = (springsPerBlock + threadsPerBlock - 1) / threadsPerBlock;
	// printf("BPE:\t%u\n", bytesPerElement);
	// printf("Block Utilization:\t%f\n", (float) bytesPerBlock / (float) maxSharedMemSize);
	// printf("EPB:\t%u\n", elementsPerBlock);
	// printf("EPB:\t%u\n", elementsPerBlock);

	short shiftskip = 20;

	SimOptions opt = {
		step_period,
		make_float4(stiffness,mu,zeta,gravity.y),
		massesPerBlock, springsPerBlock,
		numMasses, numSprings,
		shiftskip
	};
	
	uint step = 0;
	float hold_time = 0.0f;
	float mat_time = 0.0f;
	while(step_time < max_time) {
		mat_time = max(total_time-hold_time,0.0f);

		integrateBodies<<<numBlocks,threadsPerBlock,sharedMemSize>>>(
			(float4*) m_dPos[m_currentWrite], (float4*) m_dVel[m_currentWrite],
			(float4*) m_dPos[m_currentRead], (float4*) m_dVel[m_currentRead],
			(ushort2*)  m_dPairs,
			(float*) m_dLbars,
			(float4*) m_dMats,
			// (char*) m_dMatEncodings,
			(ushort*) m_dMaxStressCount, (ushort*) m_dMinStressCount,
			#ifdef FULL_STRESS
			(float*) m_dStresses, 
			(uint*) m_dSpringIDs,
			#endif
			mat_time, step, opt);

			
		gpuErrchk( cudaPeekAtLastError() );
		cudaDeviceSynchronize();
		
		std::swap(m_currentRead, m_currentWrite);
		
		step++;
		total_time += step_period;
		step_time += step_period;
	}

	cudaMemcpy(m_hPos,m_dPos[m_currentRead],numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hVel,m_dVel[m_currentRead],numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hMaxStressCount,m_dMaxStressCount,numSprings*sizeof(ushort),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hMinStressCount, m_dMinStressCount, numSprings*sizeof(ushort),cudaMemcpyDeviceToHost);
	#ifdef FULL_STRESS
	cudaMemcpy(m_hStresses,   m_dStresses,   numSprings*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hSpringIDs,   m_dSpringIDs,   numSprings*sizeof(uint), cudaMemcpyDeviceToHost);
	#endif

	for(uint i = 0; i < numMasses; i++) {
		float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
		float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};
		massBuf[i].pos = Eigen::Vector3f(pos.x,pos.y,pos.z);

		// printf("%u: {%f,%f,%f}\n",i,pos.x,pos.y,pos.z);
		massBuf[i].vel = Eigen::Vector3f(vel.x,vel.y,vel.z);
	}

	#if defined(FULL_STRESS) && defined(WRITE_STRESS)
	std::vector<std::tuple<uint, float, uint, uint>> stressHistory;

	for(uint i = 0; i < maxSprings; i++) {
		stressHistory.push_back({m_hSpringIDs[i], m_hStresses[i], m_hMaxStressCount[i], m_hMinStressCount[i]});
	}
	// std::string stressHistoryCSV = util::DataToCSV("id, stress, max count, min count",stressHistory);
	// util::WriteCSV("../z_results/stress.csv", stressHistoryCSV);
	#endif

	cudaMemcpy(m_dVel[m_currentRead], m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPos[m_currentRead], m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);

	return trackers;
}


std::vector<ElementTracker> Simulator::Allocate(const std::vector<Element>& elements) {
	std::vector<ElementTracker> trackers;

	for(uint i = 0; i < elements.size(); i++) {
		trackers.push_back(AllocateElement(elements[i]));
	}

	return trackers;
}

ElementTracker Simulator::AllocateElement(const Element& e) {
	ElementTracker tracker;

	
	tracker.mass_begin = massBuf + numMasses;
	tracker.spring_begin = springBuf + numSprings;
	tracker.offset_begin = offsetBuf + numSprings;
	tracker.mass_end = tracker.mass_begin; 
	tracker.spring_end = tracker.spring_begin;
	tracker.offset_end = tracker.offset_begin;
	uint massOffset = numMasses;
	
	numElements++;
	
	for(const Mass& m : e.masses) {
		massBuf[numMasses] = m;
		tracker.mass_end++;
		numMasses++;
	}

	uint i = 0;

	unsigned seed = rand();
	std::vector<Spring> shuffledSprings(e.springs);
	std::shuffle(shuffledSprings.begin(), shuffledSprings.end(), std::default_random_engine(seed));

	for(const Spring& s : shuffledSprings) {
		springBuf[numSprings] = s;
		tracker.spring_end++;

		offsetBuf[numSprings] = massOffset;
		tracker.offset_end++;
		numSprings++;
		i++;
	}
	
	return tracker;
}

std::vector<Element> Simulator::Collect(const std::vector<ElementTracker>& trackers) {
	std::vector<Element> elements;
	for(const ElementTracker& tracker : trackers) {
		Element e = CollectElement(tracker);
		elements.push_back(e);
	}
	return elements;
}

Element Simulator::CollectElement(const ElementTracker& tracker) {
	std::vector<Mass> result_masses;
	std::vector<Spring> result_springs;

	for(Mass* i = tracker.mass_begin; i < tracker.mass_end; i++) {
		result_masses.push_back(*i);
	}

	for(Spring* i = tracker.spring_begin; i < tracker.spring_end; i++) {
		result_springs.push_back(*i);
	}
	
	return {result_masses, result_springs};
}

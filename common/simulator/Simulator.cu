#include "Simulator.h"
#include "sim_kernel.cu"
#include "devo_kernel.cu"
#include <math.h>
#include <algorithm>
#include <functional>
#include <fstream>
#include <iostream>
#include <random>
#include "util.h"

#include <cub/device/device_segmented_radix_sort.cuh>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

template <std::size_t... Is, typename... Ts>
std::string DataToCSVImpl(const std::string& header, const std::vector<std::tuple<Ts...>>& data, std::index_sequence<Is...>)
{
    std::ostringstream os;

    // Write the header
    os << header << std::endl;

    // Write the data
    for (const auto& row : data)
    {
        bool first = true;
        ((os << (first ? first = false, "" : ","), os << std::get<Is>(row)), ...);
        os << std::endl;
    }

    return os.str();
}

template <typename... Ts>
std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data)
{
    return DataToCSVImpl(header, data, std::index_sequence_for<Ts...>());
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

	delete[] m_hMassMatEncodings;
	delete[] m_hSpringMatEncodings;
	delete[] m_hCompositeMats;

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
	cudaFree((void**) m_dRandomPairs);
	
	cudaFree((void**) m_dMassMatEncodings);
	cudaFree((void**) m_dSpringMatEncodings);
	cudaFree((void**) m_dCompositeMats);

	cudaFree((void**) m_dMaxStressCount);
	cudaFree((void**) m_dMinStressCount);
	cudaFree((void**) m_dMinStressCount_Sorted);
	cudaFree((void**) m_dStresses);
	cudaFree((void**) m_dSpringIDs);
	cudaFree((void**) m_dSpringIDs_Sorted);
}

void Simulator::Initialize(Element prototype, uint maxElements, Config::Simulator config) {
	massesPerElement = prototype.masses.size();
	springsPerElement = prototype.springs.size();

	Initialize(massesPerElement, springsPerElement, maxElements, config);
}

void Simulator::Initialize(uint massesPerElement, uint springsPerElement, uint maxElements, Config::Simulator config) {
	massesPerElement = massesPerElement;
	springsPerElement = springsPerElement;
	this->maxElements = maxElements;
	maxMasses = massesPerElement*maxElements;
	maxSprings = springsPerElement*maxElements;

	replacedSpringsPerElement = config.replaced_springs_per_element;
	maxReplaced = replacedSpringsPerElement * maxElements;
	deltaT = config.time_step;
	
	this->config = config;

	_initialize();
}


void Simulator::_initialize() { //uint maxMasses, uint maxSprings) {
	maxEnvs = 1;
	m_currentRead = 0;
	m_currentWrite = 1;
	total_time = 0.0f;
	
	if(initialized) {
		// Free CPU
		delete[] m_hPos;
		delete[] m_hVel;

		delete[] m_hLbars;
		delete[] m_hPairs;

		delete[] m_hMassMatEncodings;
		delete[] m_hSpringMatEncodings;
		delete[] m_hCompositeMats;

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
		cudaFree((void**) m_dRandomPairs);
		
		cudaFree((void**) m_dMassMatEncodings);
		cudaFree((void**) m_dSpringMatEncodings);
		cudaFree((void**) m_dCompositeMats);

		cudaFree((void**) m_dMaxStressCount);
		cudaFree((void**) m_dMinStressCount);
		cudaFree((void**) m_dMinStressCount_Sorted);
		cudaFree((void**) m_dStresses);
		cudaFree((void**) m_dSpringIDs);
		cudaFree((void**) m_dSpringIDs_Sorted);
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
	
	m_hMassMatEncodings		= new uint8_t[maxMasses];
	m_hSpringMatEncodings	= new uint8_t[maxSprings];
	m_hCompositeMats		= new float[(1<<MATERIAL_COUNT)*4];

	m_hPos 	  = new float[maxMasses*4];
    m_hVel 	  = new float[maxMasses*4];

	m_hMaxStressCount = new ushort[maxSprings];
	m_hMinStressCount  = new ushort[maxSprings];

	m_hStresses  = new float[maxSprings];
	m_hSpringIDs = new uint[maxSprings];
	
	memset(m_hPos, 0, maxMasses*4*sizeof(float));
    memset(m_hVel, 0, maxMasses*4*sizeof(float));

	memset(m_hMaxStressCount, 0, maxSprings*sizeof(ushort));
    memset(m_hMinStressCount, 0, maxSprings*sizeof(ushort));

    memset(m_hStresses, 0, maxSprings * sizeof(float));
    memset(m_hSpringIDs, 0, maxSprings * sizeof(uint));
	
    unsigned int massSizefloat4     = sizeof(float)  * 4 * maxMasses;
    unsigned int massSizeuint8_t    = sizeof(uint8_t)* 1 * maxMasses;
    unsigned int springSizeuint8_t	= sizeof(uint8_t)* 1 * maxSprings;
    unsigned int springSizeushort	= sizeof(ushort) * 1 * maxSprings;
    unsigned int springSizefloat    = sizeof(float)  * 1 * maxSprings;
    unsigned int springSizeushort2  = sizeof(ushort) * 2 * maxSprings;
    unsigned int springSizeuint     = sizeof(uint)   * 1 * maxSprings;
    unsigned int matSizefloat4     	= sizeof(float)  * 4 * (1 << MATERIAL_COUNT);
    unsigned int replaceSizeushort2 = sizeof(ushort) * 2 * maxReplaced;

	cudaMalloc((void**)&m_dVel[0], massSizefloat4);
	cudaMalloc((void**)&m_dVel[1], massSizefloat4);

	cudaMalloc((void**)&m_dPos[0], massSizefloat4);
	cudaMalloc((void**)&m_dPos[1], massSizefloat4);

	cudaMalloc((void**)&m_dPairs,  springSizeushort2);
	cudaMalloc((void**)&m_dRandomPairs,  replaceSizeushort2);
	cudaMalloc((void**)&m_dLbars,  springSizefloat);

	cudaMalloc((void**)&m_dMassMatEncodings,	massSizeuint8_t);
	cudaMalloc((void**)&m_dSpringMatEncodings,	springSizeuint8_t);
	cudaMalloc((void**)&m_dCompositeMats,		matSizefloat4);
	
	cudaMalloc((void**)&m_dMaxStressCount, springSizeushort);
	cudaMalloc((void**)&m_dMinStressCount,  springSizeushort);
	cudaMalloc((void**)&m_dMinStressCount_Sorted,  springSizeushort);

	cudaMalloc((void**)&m_dStresses,  springSizefloat);
	cudaMalloc((void**)&m_dSpringIDs,  springSizeuint);
	cudaMalloc((void**)&m_dSpringIDs_Sorted,  springSizeuint);

	switch(config.env_type) {
		case ENVIRONMENT_LAND:
			envBuf[0] = EnvironmentLand;
		case ENVIRONMENT_WATER:
			envBuf[0] = EnvironmentWater;
	}
	envCount++;
}

std::vector<ElementTracker> Simulator::SetElements(const std::vector<Element>& elements) {
	std::vector<ElementTracker> trackers;

	numMasses = 0; numSprings = 0; numElements = 0;
	for(uint i = 0; i < elements.size(); i++) {
		trackers.push_back(AllocateElement(elements[i]));
	}

	Eigen::Vector3f pos, vel;
	for(uint i = 0; i < numMasses; i++) {
		float  mass = massBuf[i].mass;
		vel  = massBuf[i].vel;
		pos = massBuf[i].pos;

		m_hMassMatEncodings[i] = massBuf[i].material.encoding;

		m_hPos[4*i]   = pos.x();
		m_hPos[4*i+1] = pos.y();
		m_hPos[4*i+2] = pos.z();
		m_hPos[4*i+3] = mass;
		
		m_hVel[4*i]   = vel.x();
		m_hVel[4*i+1] = vel.y();
		m_hVel[4*i+2] = vel.z();
	}

	for(uint i = 0; i < (1 << MATERIAL_COUNT); i++) {
		Material mat = materials::decode(i);
		m_hCompositeMats[4*i] = mat.k;
		m_hCompositeMats[4*i+1] = mat.dL0;
		m_hCompositeMats[4*i+2] = mat.omega;
		m_hCompositeMats[4*i+3] = mat.phi;
	}

	for(uint i = 0; i < numSprings; i++) {
		float    lbar     	 = springBuf[i].mean_length;
		uint	 left     	 = springBuf[i].m0,
			     right	  	 = springBuf[i].m1;
		uint8_t	 matEncoding = springBuf[i].material.encoding;

		m_hSpringIDs[i] = i;
		m_hPairs[2*i]   = left;
		m_hPairs[2*i+1] = right;
		m_hLbars[i] 	= lbar;

		m_hSpringMatEncodings[i] = matEncoding;

		m_hPairs[2*i]   = left;
		m_hPairs[2*i+1] = right;
		m_hLbars[i] 	= lbar;
	}

	cudaMemcpy(m_dVel[m_currentRead], m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPos[m_currentRead], m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPairs,  m_hPairs,  numSprings *2*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dLbars,  m_hLbars,  numSprings  * sizeof(float), cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dMassMatEncodings,		m_hMassMatEncodings,   	 numMasses  * sizeof(uint8_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringMatEncodings,   m_hSpringMatEncodings,   numSprings * sizeof(uint8_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dCompositeMats,   m_hCompositeMats,   (1<<MATERIAL_COUNT) *4*sizeof(float), cudaMemcpyHostToDevice);

	#ifdef FULL_STRESS
	cudaMemcpy(m_dStresses,   m_hStresses,   numSprings*sizeof(float), cudaMemcpyHostToDevice);
	#endif

	cudaMemcpy(m_dSpringIDs,   m_hSpringIDs,   numSprings*sizeof(uint), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMaxStressCount,   m_hMaxStressCount,   numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMinStressCount,    m_hMinStressCount,    numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	gpuErrchk( cudaPeekAtLastError() );

	return trackers;
}

void Simulator::Simulate(float sim_duration, bool trackStresses) {
	float simTimeRemaining = sim_duration;
	
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
	uint bytesPerMaterial = sizeof(float4);
	// Must equal 1 for proper max/min spring calculation
	uint elementsPerBlock = min((maxSharedMemSize - (1<<MATERIAL_COUNT)*bytesPerMaterial) / bytesPerElement, numElements);
	uint massesPerBlock = massesPerElement * elementsPerBlock;
	uint springsPerBlock = springsPerElement * elementsPerBlock;
	uint sharedMemSize = massesPerBlock * bytesPerMass + (1<<MATERIAL_COUNT)*bytesPerMaterial;
	uint numBlocks = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	assert(sharedMemSize < maxSharedMemSize);
	// uint bytesPerBlock = elementsPerBlock * bytesPerElement;
	// int numBlocks = (springsPerBlock + threadsPerBlock - 1) / threadsPerBlock;
	// printf("BPE:\t%u\n", bytesPerElement);
	// printf("Block Utilization:\t%f\n", (float) bytesPerBlock / (float) maxSharedMemSize);
	// printf("EPB:\t%u\n", elementsPerBlock);
	// printf("EPB:\t%u\n", elementsPerBlock);

	short shiftskip = 20;

	SimOptions opt = {
		deltaT,
		massesPerBlock, springsPerBlock,
		numMasses, numSprings,
		MATERIAL_COUNT,
		shiftskip,
		envBuf[0]
	};
	
	uint step_count = 0;

	#ifdef DEBUG_TRACE
		std::vector<std::tuple<unsigned int, float, float, float, float, float, float, float>> massTrace;
		static int sim_run = 0;
	#endif

	
	while(simTimeRemaining > 0.0f) {
		integrateBodies<<<numBlocks,simThreadsPerBlock,sharedMemSize>>>(
			(float4*) m_dPos[m_currentWrite], (float4*) m_dVel[m_currentWrite],
			(float4*) m_dPos[m_currentRead], (float4*) m_dVel[m_currentRead],
			(ushort2*)  m_dPairs, (uint8_t*) m_dSpringMatEncodings,  (float*) m_dLbars,
			(float4*) m_dCompositeMats,
			total_time, step_count, opt);

		gpuErrchk( cudaPeekAtLastError() );
		cudaDeviceSynchronize();
			
		std::swap(m_currentRead, m_currentWrite);

		#ifdef DEBUG_TRACE
			if(step_count % 20 == 0) {
				cudaMemcpy(m_hPos,m_dPos[m_currentRead],numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
				cudaMemcpy(m_hVel,m_dVel[m_currentRead],numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);

				for(unsigned int i = 0; i < numMasses; i++) {
					float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
					float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};

					massTrace.push_back({ i, total_time, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z });
				}
			}


			
		#endif
		
		step_count++;
		total_time += deltaT;
		simTimeRemaining -= deltaT;
	}

	#ifdef DEBUG_TRACE
		std::string massTraceCSV = DataToCSV<unsigned int, float, float, float, float, float, float, float>("id, time, x, y, z, vx, vy, vz",massTrace);
		util::WriteCSV(std::string("sim_trace_") + std::to_string(sim_run) + std::string(".csv"), "/mnt/vault/z_results", massTraceCSV);
		sim_run++;
	#endif
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

		assert(!isnan(pos.x) && !isnan(pos.y) && !isnan(pos.z));

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

void key_value_sort(ushort* d_keys_in, ushort* d_keys_out, uint* d_values_in, uint* d_values_out, uint items_per_segment, uint num_segments) {
    // Determine number of items
    int num_items = num_segments * items_per_segment;

    // Allocate memory on device for offsets
    int* h_offsets = new int[num_segments+1];
    for(uint i = 0; i < num_segments+1; i++) {
        h_offsets[i] = (int) items_per_segment*i;
    }

    int* d_offsets;
    cudaMalloc(&d_offsets, (num_segments+1) * sizeof(int));
    cudaMemcpy(d_offsets, h_offsets, (num_segments+1) *sizeof(int), cudaMemcpyHostToDevice);

    // Determine temporary storage size
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceSegmentedRadixSort::SortPairsDescending(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        num_items, num_segments, d_offsets, d_offsets+1);

    // Allocate temporary storage
    cudaMalloc(&d_temp_storage, temp_storage_bytes);

    // Run sorting operation
    cub::DeviceSegmentedRadixSort::SortPairsDescending(
        d_temp_storage, temp_storage_bytes,
        d_keys_in, d_keys_out, d_values_in, d_values_out,
        num_items, num_segments, d_offsets, d_offsets+1);

    delete[] h_offsets;
    cudaFree(d_offsets);
    cudaFree(d_temp_storage);
}

__global__ void
inline printMinStressCount(uint numSprings, ushort* minStressCount, uint* springId) {
	uint tid    = threadIdx.x;
	uint stride = blockDim.x;

	for(uint i = tid; i < numSprings; i+=stride) {
		printf("%u: Stress %u, Spring %u\n", i, minStressCount[i], springId[i]);
	}

}

void Simulator::Devo() {
	static int seed = 0;

	uint threadsPerBlock = 256;
	uint numReplacedSprings = replacedSpringsPerElement * numElements;
	uint numBlocks = (numReplacedSprings+threadsPerBlock - 1) / threadsPerBlock;
	randomIntegerPairKernel<<<numBlocks, threadsPerBlock>>>(numReplacedSprings, (ushort2*) m_dRandomPairs,0,massesPerElement-1,seed);

    key_value_sort(m_dMinStressCount, m_dMinStressCount_Sorted, m_dSpringIDs, m_dSpringIDs_Sorted, springsPerElement, numElements);
	// printMinStressCount<<<1, threadsPerBlock>>>(threadsPerBlock*3, m_dMinStressCount_Sorted, m_dSpringIDs_Sorted);

	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );

	uint bytesPerMaterial = sizeof(float4);
	uint sharedMemSize = (1<<MATERIAL_COUNT)*bytesPerMaterial;
	numBlocks = (numReplacedSprings + threadsPerBlock - 1) / threadsPerBlock;
	
	DevoOptions opt = {
		numReplacedSprings,
		numSprings,
		springsPerElement,
		massesPerElement,
		replacedSpringsPerElement,
		MATERIAL_COUNT
	};
			
	replaceSprings<<<numBlocks, devoThreadsPerBlock, sharedMemSize>>>(
			(ushort2*) m_dPairs, (uint8_t*) m_dMassMatEncodings,
			(float4*) m_dPos[m_currentRead], (float*) m_dLbars,
			(uint8_t*) m_dSpringMatEncodings,
			(uint*) m_dSpringIDs_Sorted, (ushort2*) m_dRandomPairs, 
			(float4*) m_dCompositeMats, total_time,
			opt
		);
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );


	cudaMemset(m_dMinStressCount, 0, numSprings * sizeof(ushort));
	cudaMemset(m_dMaxStressCount, 0, numSprings * sizeof(ushort));
	
	if(config.visual) {
		cudaMemcpy(m_hSpringMatEncodings, m_dSpringMatEncodings, numSprings*sizeof(uint8_t), cudaMemcpyDeviceToHost);
		cudaMemcpy(m_hPairs, m_dPairs, numSprings*2*sizeof(ushort), cudaMemcpyDeviceToHost);
		cudaMemcpy(m_hLbars, m_dLbars, numSprings * sizeof(float),  cudaMemcpyDeviceToHost);

		for(uint i = 0; i < numSprings; i++) {
			springBuf[i].m0 = m_hPairs[2*i];
			springBuf[i].m1 = m_hPairs[2*i+1];
			springBuf[i].mean_length = m_hLbars[i];
			springBuf[i].material = materials::decode(m_hSpringMatEncodings[i]);
		}
	}

	seed++;
}

/*
uint partition(ushort *minStressCount, ushort *spring_ids, uint start, uint end) {
	uint pivot = minStressCount[start];
    uint count = 0, pivot_index, i = start, j = end;

    for (int i = start + 1; i <= end; i++) {
        if (arr[i] <= pivot)
            count++;
    }
 
    pivot_index = start + count;
    std::swap(minStressCount[pivot_index], minStressCount[start]);
	std::swap(spring_ids[pivot_index], spring_ids[start]);
 
    while (i < pivot_index && j > pivot_index) {
        while (minStressCount[i] <= pivot) {
            i++;
        }
        while (minStressCount[j] > pivot) {
            j--;
        }
        if (i < pivot_index && j > pivot_index) {
            std::swap(minStressCount[i++], minStressCount[j--]);
			std::swap(spring_ids[i++], spring_ids[j--]);
        }
    }
 
    return pivot_index;
}

void quickSort(ushort *minStressCount, ushort *spring_ids, uint start, uint end) {
    if (start >= end)
        return;
 
    uint partition = partition(minStressCount, spring_ids, start, end);
    quickSort(minStressCount, spring_ids, partition-1);
    quickSort(minStressCount, spring_ids, partition+1, end);
}

void linkedSort(ushort minStressCount, ushort *spring_ids, uint num_springs) {
	quicksort(ushort2 minStressCount, ushort *spring_ids, 0, num_springs-1)
}

void Simulator::replaceSprings(ushort2 *__restrict__ pairs, ushort *__restrict__ maxStressCount, ushort *__restrict__ minStressCount) {
	ushort2 rand1, rand2;
	//TODO: Properly alloc memory for this
	//TODO: Find and pass num_springs
	ushort spring_ids[num_springs];
	for(uint i = 0; i < num_springs; i++){
		spring_ids[i] = i;
	}
	
	linkedSort(minStressCount, spring_ids, num_springs);

	for(uint i = 0; i < replace_amount; i++) {
		rand1 = rand() % num_springs;
		do {
			rand2 = rand() % num_springs;
		} while(rand1 == rand2)
		pairs[spring_ids[i]].x = rand1;
		pairs[spring_ids[i]].y = rand2;
		//TODO: Calculate New Rest Length - Might need to pass in masses for that
	}
}*/
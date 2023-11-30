#include "Simulator.h"
#include <math.h>
#include <algorithm>
#include <random>
#include <map>
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
std::string DataToCSVImpl(const std::string& header, const std::vector<std::tuple<Ts...>>& data, std::index_sequence<Is...>, bool writeHeader)
{
    std::ostringstream os;

    // Write the header
	if(writeHeader) os << header << std::endl;

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
std::string DataToCSV(const std::string& header, const std::vector<std::tuple<Ts...>>& data, bool writeHeader=true)
{
    return DataToCSVImpl(header, data, std::index_sequence_for<Ts...>(), writeHeader);
}

void Simulator::freeMemory() {
	// Free CPU
	delete[] massBuf;
	delete[] springBuf;
	delete[] faceBuf;
	delete[] cellBuf;
	delete[] envBuf;

	delete[] m_hCompositeMats_encoding;
	delete[] m_hCompositeMats_id;
	
	delete[] m_hPos;
	delete[] m_hVel;
	delete[] m_hMassMatEncodings;

	delete[] m_hPairs;
	delete[] m_hSpringMatEncodings;
	delete[] m_hSpringMatIds;	gpuErrchk( cudaPeekAtLastError() );

	delete[] m_hLbars;
	delete[] m_hSpringIDs;
	delete[] m_hSpringStresses;
	
	delete[] m_hFaces;

	delete[] m_hCells;
	delete[] m_hVbars;
	delete[] m_hMats;
	delete[] m_hCellStresses;

	// Free GPU
	cudaFree((void**) m_dData.dPos);
	cudaFree((void**) m_dData.dNewPos);
	cudaFree((void**) m_dData.dVel);
	cudaFree((void**) m_dData.dMassMatEncodings);

	cudaFree((void**) m_dData.dPairs);
	cudaFree((void**) m_dData.dSpringMatEncodings);
	cudaFree((void**) m_dData.dSpringMatIds);
	cudaFree((void**) m_dData.dLbars);
	cudaFree((void**) m_dData.dSpringIDs);
	cudaFree((void**) m_dData.dSpringStresses);
	cudaFree((void**) m_dData.dRandomPairs);
	cudaFree((void**) m_dData.dSpringStresses_Sorted);
	cudaFree((void**) m_dData.dSpringIDs_Sorted);

	cudaFree((void**) m_dData.dFaces);

	cudaFree((void**) m_dData.dCells);
	cudaFree((void**) m_dData.dVbars);
	cudaFree((void**) m_dData.dMats);
	cudaFree((void**) m_dData.dCellStresses);
}

Simulator::~Simulator() {
	if(initialized) freeMemory();
}

void Simulator::Initialize(Config::Simulator config) {
	m_replacedSpringsPerElement = config.replaced_springs_per_element;
	m_deltaT = config.time_step;
	m_config = config;

	_initialize();
}


void Simulator::_initialize() { //uint maxMasses, uint maxSprings) {
	maxEnvs = 1;
	m_total_time = 0.0f;
	
	if(initialized) freeMemory();
	initialized = true;
	
	massBuf   = new Mass[maxMasses];
	springBuf = new Spring[maxSprings];
	faceBuf   = new Face[maxFaces];
	cellBuf   = new Cell[maxCells];
	envBuf 	  = new Environment[1];

	m_hCompositeMats_encoding	= new float[COMPOSITE_COUNT*4];
	m_hCompositeMats_id			= new float[COMPOSITE_COUNT*4];

	m_hPos 	  = new float[maxMasses*4];
	m_hVel 	  = new float[maxMasses*4];
	m_hMassMatEncodings		= new uint32_t[maxMasses];

	m_hPairs  = new ushort[maxSprings*2];
	m_hSpringMatEncodings	= new uint32_t[maxSprings];
	m_hSpringMatIds			= new uint8_t[maxSprings];
	m_hLbars  = new float[maxSprings];
	m_hSpringIDs = new uint[maxSprings];
	m_hSpringStresses  = new float[maxSprings];

	m_hFaces  = new ushort[maxFaces*4];

	m_hCells  = new ushort[maxCells*4];
	m_hVbars  = new float[maxCells];
	m_hMats  = new float[maxCells*4];
	m_hCellStresses  = new float[maxCells];

	memset(m_hPos, 0, maxMasses*4*sizeof(float));
	memset(m_hVel, 0, maxMasses*4*sizeof(float));

	memset(m_hSpringStresses, 0, maxSprings * sizeof(float));
    memset(m_hCellStresses, 0, maxCells * sizeof(float));

    // memset(m_hSpringIDs, 0, maxSprings * sizeof(uint));
    // memset(m_hSpringMatEncodings, 0, maxSprings * sizeof(uint));
	
    unsigned int massSizefloat4     = sizeof(float)    * 4 * maxMasses;
    unsigned int massSizeuint32_t   = sizeof(uint32_t) * 1 * maxMasses;
    unsigned int springSizeuint32_t	= sizeof(uint32_t) * 1 * maxSprings;
    unsigned int springSizeuint8_t	= sizeof(uint8_t)  * 1 * maxSprings;
    unsigned int springSizefloat    = sizeof(float)    * 1 * maxSprings;
    unsigned int springSizeushort2  = sizeof(ushort)   * 2 * maxSprings;
    unsigned int springSizeuint     = sizeof(uint)     * 1 * maxSprings;
    unsigned int replaceSizeushort2 = sizeof(ushort)   * 2 * maxReplaced;
    unsigned int faceSizeushort4    = sizeof(ushort)   * 4 * maxFaces;
    unsigned int cellSizeushort4    = sizeof(ushort)   * 4 * maxCells;
	unsigned int cellSizefloat      = sizeof(float)    * 1 * maxCells;
    unsigned int cellSizefloat4     = sizeof(float)    * 4 * maxCells;

	cudaMalloc((void**)&m_dData.dPos, massSizefloat4);
	cudaMalloc((void**)&m_dData.dNewPos, massSizefloat4);
	cudaMalloc((void**)&m_dData.dVel, massSizefloat4);
	cudaMalloc((void**)&m_dData.dMassMatEncodings,	massSizeuint32_t);
	gpuErrchk( cudaPeekAtLastError() );

	cudaMalloc((void**)&m_dData.dPairs,  springSizeushort2);
	cudaMalloc((void**)&m_dData.dSpringMatEncodings,	springSizeuint32_t);
	cudaMalloc((void**)&m_dData.dSpringMatIds,		springSizeuint8_t);
	cudaMalloc((void**)&m_dData.dLbars,  springSizefloat);
	cudaMalloc((void**)&m_dData.dSpringIDs,  springSizeuint);
	cudaMalloc((void**)&m_dData.dSpringStresses,  springSizefloat);
	cudaMalloc((void**)&m_dData.dRandomPairs,  replaceSizeushort2);
	cudaMalloc((void**)&m_dData.dSpringIDs_Sorted,  springSizeuint);
	cudaMalloc((void**)&m_dData.dSpringStresses_Sorted,  springSizefloat);

	cudaMalloc((void**)&m_dData.dFaces,  faceSizeushort4);
	
	cudaMalloc((void**)&m_dData.dCells,  cellSizeushort4);
	cudaMalloc((void**)&m_dData.dVbars,  cellSizefloat);
	cudaMalloc((void**)&m_dData.dMats,  cellSizefloat4);
	cudaMalloc((void**)&m_dData.dCellStresses,  cellSizefloat);

	switch(m_config.env_type) {
		case ENVIRONMENT_LAND:
			envBuf[0] = EnvironmentLand;
		case ENVIRONMENT_WATER:
			envBuf[0] = EnvironmentWater;
	}
	envCount++;
}

ElementTracker Simulator::SetElement(const Element& element) {
	std::vector<Element> elements = {element};
	std::vector<ElementTracker> trackers = SetElements(elements);
	return trackers[0];
}

std::vector<ElementTracker> Simulator::SetElements(const std::vector<Element>& elements) {
	std::vector<ElementTracker> trackers;

	// cudaFuncAttributes attr;
	// cudaFuncGetAttributes(&attr, integrateBodiesStresses);
	// simThreadsPerBlock = attr.maxThreadsPerBlock;
	// assert( attr.numRegs <= 32768 );
	// cudaFuncGetAttributes(&attr, integrateBodies);
	// if(attr.maxThreadsPerBlock < simThreadsPerBlock)
	// 	simThreadsPerBlock = attr.maxThreadsPerBlock;
	
	// assert( attr.numRegs <= 32768 );
	uint largestElementBoundaryMasses = 0;
	uint largestElementSprings = 0;
	uint largestElementFaces = 0;
	uint largestElementCells = 0;
	for(auto& e : elements) {
		if(e.boundaryCount > largestElementBoundaryMasses) largestElementBoundaryMasses = e.boundaryCount;
		if(e.springs.size() > largestElementSprings) largestElementSprings = e.springs.size();
		if(e.faces.size() > largestElementFaces) largestElementFaces = e.faces.size();
		if(e.cells.size() > largestElementCells) largestElementCells = e.cells.size();
	}

	maxElements = elements.size();
	maxReplaced = m_replacedSpringsPerElement * maxElements;
	massesPerElement = elements[0].masses.size();
	boundaryMassesPerElement = largestElementBoundaryMasses;
	springsPerElement = largestElementSprings;
	facesPerElement = largestElementFaces;
	cellsPerElement = largestElementCells;
	maxMasses = massesPerElement*maxElements;
	maxSprings = largestElementSprings*maxElements;
	maxFaces = largestElementFaces*maxElements;
	maxCells = largestElementCells*maxElements;

	_initialize();

	numElements = 0; numMasses = 0; numSprings = 0; numFaces = 0; numCells = 0;
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

	uint32_t encoding = 0x01u;
	Material mat = materials::decode(encoding);
	m_hCompositeMats_encoding[0] = mat.k;
	m_hCompositeMats_encoding[1] = mat.dL0;
	m_hCompositeMats_encoding[2] = mat.omega;
	m_hCompositeMats_encoding[3] = mat.phi;

	uint idx;
	for(uint i = 1; i < MATERIAL_COUNT; i++) {
		for(uint j = i; j < MATERIAL_COUNT; j++) {
			encoding = (0x01u << i) | (0x01u << j);
			idx = materials::encodedCompositeIdx(encoding);
			mat = materials::decode(encoding);
			m_hCompositeMats_encoding[4*idx] = mat.k;
			m_hCompositeMats_encoding[4*idx+1] = mat.dL0;
			m_hCompositeMats_encoding[4*idx+2] = mat.omega;
			m_hCompositeMats_encoding[4*idx+3] = mat.phi;
		}
	}

	for(uint i = 0; i < COMPOSITE_COUNT; i++) {
		Material mat = materials::id_lookup(i);
		m_hCompositeMats_id[4*i] = mat.k;
		m_hCompositeMats_id[4*i+1] = mat.dL0;
		m_hCompositeMats_id[4*i+2] = mat.omega;
		m_hCompositeMats_id[4*i+3] = mat.phi;
	}

	for(uint i = 0; i < numSprings; i++) {
		float    lbar        = springBuf[i].mean_length;
		ushort	 m0          = springBuf[i].m0,
			     m1	         = springBuf[i].m1;
		uint8_t	 matId 		 = springBuf[i].material.id;
		uint32_t matEncoding = springBuf[i].material.encoding;

		m_hSpringIDs[i] = i;
		m_hPairs[2*i]   = m0;
		m_hPairs[2*i+1] = m1;
		m_hLbars[i] 	= lbar;

		m_hSpringMatIds[i] = matId;
		m_hSpringMatEncodings[i] = matEncoding;
	}

	for(uint i = 0; i < numFaces; i++) {
		ushort   m0          = faceBuf[i].m0,
			     m1          = faceBuf[i].m1,
			     m2          = faceBuf[i].m2;

		m_hFaces[4*i]   = m0;
		m_hFaces[4*i+1] = m1;
		m_hFaces[4*i+2] = m2;
	}

	for(uint i = 0; i < numCells; i++) {
		ushort   m0          = cellBuf[i].m0,
			     m1          = cellBuf[i].m1,
			     m2          = cellBuf[i].m2,
			     m3          = cellBuf[i].m3;
		float    vbar        = cellBuf[i].mean_volume;
        Material mat         = cellBuf[i].material;

		m_hCells[4*i]   = m0;
		m_hCells[4*i+1] = m1;
		m_hCells[4*i+2] = m2;
		m_hCells[4*i+3] = m3;

		m_hMats[4*i]   = mat.k;
		m_hMats[4*i+1] = mat.dL0;
		m_hMats[4*i+2] = mat.omega;
		m_hMats[4*i+3] = mat.phi;

		m_hVbars[i] = vbar;
	}

	setCompositeMats_id(m_hCompositeMats_id, COMPOSITE_COUNT);
	gpuErrchk( cudaPeekAtLastError() );
	setCompositeMats_encoding(m_hCompositeMats_encoding, COMPOSITE_COUNT);
	gpuErrchk( cudaPeekAtLastError() );

	cudaMemcpy(m_dData.dPos, m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dVel, m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dMassMatEncodings,		m_hMassMatEncodings,   	 numMasses  * sizeof(uint32_t), cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dData.dPairs,  				m_hPairs			  , numSprings*2*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dSpringMatEncodings,	m_hSpringMatEncodings , numSprings * sizeof(uint32_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dSpringMatIds,   		m_hSpringMatIds		  , numSprings * sizeof(uint8_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dLbars,  				m_hLbars			  , numSprings * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dSpringIDs,   			m_hSpringIDs		  , numSprings * sizeof(uint), cudaMemcpyHostToDevice);
	cudaMemset(m_dData.dSpringStresses,  		0.0f				  , numSprings * sizeof(float));
	
	cudaMemcpy(m_dData.dFaces,  m_hFaces,  numFaces *4*sizeof(ushort),  cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dData.dCells,  		m_hCells,  numCells*4*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dVbars,  		m_hVbars,  numCells*1*sizeof(float) ,  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dData.dMats ,  		m_hMats ,  numCells*4*sizeof(float) ,  cudaMemcpyHostToDevice);
	cudaMemset(m_dData.dCellStresses, 0.0f	,  numCells*1*sizeof(float));

	gpuErrchk( cudaPeekAtLastError() );

	return trackers;
}

void Simulator::Simulate(float sim_duration, bool trackStresses, bool trace, std::string tracefile) {
	float simTimeRemaining = sim_duration;
	
	// /*
	// Notes on SM resources:
	// thread blocks	: 8
	// threads			: 2048
	// registers		: 65536
	// shared mem		: 49152
	// */

	uint elementsPerBlock = 1;
	m_massesPerBlock = massesPerElement * elementsPerBlock;
	m_springsPerBlock = springsPerElement * elementsPerBlock;
	m_facesPerBlock = facesPerElement * elementsPerBlock;
	m_cellsPerBlock = cellsPerElement * elementsPerBlock;
	uint boundaryMassesPerBlock = boundaryMassesPerElement * elementsPerBlock;
	m_numBlocksSim = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	// printf("%u / %u\n", m_massesPerBlock, (uint) (maxSharedMemSize - COMPOSITE_COUNT*bytesPerMaterial - sizeof(ushort)*2048) / (bytesPerMass));
	// printf("%u / %u\n", m_sharedMemSizeSim, maxSharedMemSize);

	// assert(m_sharedMemSizeSim <= maxSharedMemSize);

	// uint bytesPerBoundaryMass = sizeof(float3) + sizeof(float3) + sizeof(float3);
	// uint dragSharedMemorySize = boundaryMassesPerBlock * bytesPerBoundaryMass;
	// uint numDragBlocks = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	// // printf("%u / %u\n", m_massesPerBlock, (uint) (maxSharedMemSize - COMPOSITE_COUNT*bytesPerMaterial - sizeof(ushort)*2048) / (bytesPerMass));
	// // printf("%u / %u\n", m_sharedMemSizeSim, maxSharedMemSize);

	// assert(dragSharedMemorySize <= maxSharedMemSize);

	short shiftskip = 20;

	SimOptions opt = {
		m_deltaT,
		m_massesPerBlock, m_springsPerBlock, m_facesPerBlock, m_cellsPerBlock,
		boundaryMassesPerBlock,
		numMasses, numSprings, numFaces, numCells,
		COMPOSITE_COUNT,
		shiftskip,
		envBuf[0].drag,
		envBuf[0].damping,
		1.0,
		0.2
	};
	
	uint step_count = 0;

	static std::map<std::string, int> trace_hist;
	if(trace) {
		if(!trace_hist[tracefile]) trace_hist[tracefile] = 0;
	}

	std::vector<std::tuple<unsigned int, float, float, float, float, float, float, float>> massTrace;
	
	setSimOpts(opt);
	
	while(simTimeRemaining > 0.0f) {
		integrateBodies(m_dData, numElements, opt, m_total_time, step_count, trackStresses);
		gpuErrchk( cudaPeekAtLastError() );

		if(trace) {
			if(step_count % 20 == 0) {
				cudaMemcpy(m_hPos,m_dData.dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
				cudaMemcpy(m_hVel,m_dData.dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	
				for(unsigned int i = 0; i < numMasses; i++) {
					float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
					float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};
	
					massTrace.push_back({ i, m_total_time, pos.x, pos.y, pos.z, vel.x, vel.y, vel.z });
				}
			}
		}
		
		step_count++;
		m_total_time += m_deltaT;
		simTimeRemaining -= m_deltaT;
	}

	if(trace) {
		bool firstTrace = true;
		if(trace_hist[tracefile] > 0) firstTrace = false;
		std::string massTraceCSV = DataToCSV<unsigned int, float, float, float, float, float, float, float>("id, time, x, y, z, vx, vy, vz",massTrace, firstTrace);
		util::WriteCSV(tracefile, "/mnt/vault/evo-devo/z_results", massTraceCSV, !firstTrace);
		trace_hist[tracefile]++;
	}
}

ElementTracker Simulator::AllocateElement(const Element& e) {
	ElementTracker tracker;

	tracker.mass_begin = massBuf + numMasses;
	tracker.spring_begin = springBuf + numSprings;
	tracker.mass_end = tracker.mass_begin; 
	tracker.spring_end = tracker.spring_begin;
	
	for(const Mass& m : e.masses) {
		massBuf[numMasses] = m;
		tracker.mass_end++;
		numMasses++;
	}

	
	// unsigned seed = rand();
	// std::vector<Spring> shuffledSprings(e.springs);
	// std::shuffle(shuffledSprings.begin(), shuffledSprings.end(), std::default_random_engine(seed));

	// for(const Spring& s : shuffledSprings) {
	uint count = 0;
	for(const Spring& s : e.springs) {
		springBuf[numSprings] = s;
		tracker.spring_end++;

		numSprings++;
		count++;
	}

	// fill up springs to max spring size
	// TODO: variable spring count per robot
	for( ; count < springsPerElement; count++) {
		springBuf[numSprings] = {0,0,0.0f,0.0f,materials::air};
		numSprings++;
	}

	count = 0;
	for(const Face& f : e.faces) {
		faceBuf[numFaces] = f;

		numFaces++;
		count++;
	}

	// fill up faces to max faces size
	// TODO: variable face count per robot
	for( ; count < facesPerElement; count++) {
		faceBuf[numFaces] = {0,0,0};
		numFaces++;
	}

	count = 0;
	for(const Cell& c : e.cells) {
		cellBuf[numCells] = c;

		numCells++;
		count++;
	}

	// fill up cells to max cell size
	// TODO: variable cell count per robot
	for( ; count < cellsPerElement; count++) {
		cellBuf[numCells] = {0,0,0,0,0.0f,materials::air};
		numCells++;
	}
	
	numElements++;
	
	return tracker;
}

Element Simulator::Collect(const ElementTracker& tracker) {
	std::vector<ElementTracker> trackers = {tracker};
	std::vector<Element> results = Collect(trackers);
	return results[0];
}

std::vector<Element> Simulator::Collect(const std::vector<ElementTracker>& trackers) {
	cudaMemcpy(m_hPos,m_dData.dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hVel,m_dData.dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hSpringStresses,   m_dData.dSpringStresses,   numSprings*sizeof(float), cudaMemcpyDeviceToHost);

	cudaMemcpy(m_hSpringMatEncodings, m_dData.dSpringMatEncodings, numSprings*sizeof(uint32_t), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hPairs, m_dData.dPairs, numSprings*2*sizeof(ushort), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hLbars, m_dData.dLbars, numSprings * sizeof(float),  cudaMemcpyDeviceToHost);

	
	for(uint i = 0; i < numMasses; i++) {
		float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
		float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};
		assert(!isnan(pos.x) && !isnan(pos.y) && !isnan(pos.z));
		
		massBuf[i].pos = Eigen::Vector3f(pos.x,pos.y,pos.z);
		massBuf[i].vel = Eigen::Vector3f(vel.x,vel.y,vel.z);
	}

	for(uint i = 0; i < numSprings; i++) {
		springBuf[i].m0 = m_hPairs[2*i];
		springBuf[i].m1 = m_hPairs[2*i+1];
		springBuf[i].mean_length = m_hLbars[i];
		springBuf[i].material = materials::decode(m_hSpringMatEncodings[i]);
	}

	#if defined(FULL_STRESS) && defined(WRITE_STRESS)
	std::vector<std::tuple<uint, float, uint, uint>> stressHistory;

	for(uint i = 0; i < maxSprings; i++) {
		stressHistory.push_back({m_hSpringIDs[i], m_hStresses[i], m_hMaxStressCount[i], m_hMinStressCount[i]});
	}
	std::string stressHistoryCSV = util::DataToCSV("id, stress, max count, min count",stressHistory);
	util::WriteCSV("stress.csv","../z_results/", stressHistoryCSV);
	#endif


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

void key_value_sort(float* d_keys_in, float* d_keys_out, uint* d_values_in, uint* d_values_out, uint items_per_segment, uint num_segments) {
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

__global__ inline void printStress(uint numSprings, float* stress, uint* springId) {
	uint stride = blockDim.x*gridDim.x;
	for(uint i = blockIdx.x*blockDim.x + threadIdx.x; i < numSprings; i += stride) {
		printf("%u: Stress %f, Spring %u\n", i, stress[i], springId[i]);
	}
}

void Simulator::Devo() {
	static int seed = 0;

	uint numReplacedSprings = m_replacedSpringsPerElement * numElements;

    key_value_sort(m_dData.dSpringStresses, m_dData.dSpringStresses_Sorted, m_dData.dSpringIDs, m_dData.dSpringIDs_Sorted, springsPerElement, numElements);

	getRandomInterPairs(numReplacedSprings, m_dData.dRandomPairs, 0, massesPerElement-1, seed);
	
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );

	DevoOptions opt = {
		numReplacedSprings,
		numSprings,
		springsPerElement,
		massesPerElement,
		m_replacedSpringsPerElement,
		COMPOSITE_COUNT
	};
	
	setDevoOpts(opt);
	devoBodies(m_dData, opt, m_total_time);
	
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );

	seed++;
}
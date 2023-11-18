#include "Simulator.h"
#include "sim_kernel.cu"
#include "devo_kernel.cu"
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
	delete[] offsetBuf;
	delete[] envBuf;
	
	delete[] m_hPos;
	delete[] m_hVel;
	delete[] m_hLbars;
	delete[] m_hPairs;

	delete[] m_hMassMatEncodings;
	delete[] m_hSpringMatEncodings;
	delete[] m_hSpringMatIds;
	delete[] m_hCompositeMats_encoding;
	delete[] m_hCompositeMats_id;

	delete[] m_hMaxStressCount;
	delete[] m_hMinStressCount;

	delete[] m_hStresses;
	delete[] m_hSpringIDs;

	// Free GPU
	cudaFree((void**) m_dPos);
	cudaFree((void**) m_dVel);

	cudaFree((void**) m_dLbars);
	cudaFree((void**) m_dPairs);
	cudaFree((void**) m_dRandomPairs);
	
	cudaFree((void**) m_dMassMatEncodings);
	cudaFree((void**) m_dSpringMatEncodings);
	cudaFree((void**) m_dSpringMatIds);
	cudaFree((void**) m_dCompositeMats_encoding);
	cudaFree((void**) m_dCompositeMats_id);

	cudaFree((void**) m_dMaxStressCount);
	cudaFree((void**) m_dMinStressCount);
	cudaFree((void**) m_dMinStressCount_Sorted);
	cudaFree((void**) m_dStresses);
	cudaFree((void**) m_dSpringIDs);
	cudaFree((void**) m_dSpringIDs_Sorted);
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

	cudaFuncAttributes attr;
	cudaFuncGetAttributes(&attr, integrateBodiesStresses);
	simThreadsPerBlock = attr.maxThreadsPerBlock;
	assert( attr.numRegs <= 32768 );
	cudaFuncGetAttributes(&attr, integrateBodies);
	if(attr.maxThreadsPerBlock < simThreadsPerBlock)
		simThreadsPerBlock = attr.maxThreadsPerBlock;
	
	assert( attr.numRegs <= 32768 );
	
	if(initialized) freeMemory();
	initialized = true;
	
	massBuf   = new Mass[maxMasses];
	springBuf = new Spring[maxSprings];
	offsetBuf = new uint[maxSprings];
	envBuf 	  = new Environment[1];

	m_hLbars  = new float[maxSprings];
	m_hPairs  = new ushort[maxSprings*2];
	
	m_hMassMatEncodings		= new uint32_t[maxMasses];
	m_hSpringMatEncodings	= new uint32_t[maxSprings];
	m_hSpringMatIds			= new uint8_t[maxSprings];
	m_hCompositeMats_encoding	= new float[COMPOSITE_COUNT*4];
	m_hCompositeMats_id			= new float[COMPOSITE_COUNT*4];

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
	
    unsigned int massSizefloat4     = sizeof(float)    * 4 * maxMasses;
    unsigned int massSizeuint32_t   = sizeof(uint32_t) * 1 * maxMasses;
    unsigned int springSizeuint32_t	= sizeof(uint32_t) * 1 * maxSprings;
    unsigned int springSizeuint8_t	= sizeof(uint8_t)  * 1 * maxSprings;
    unsigned int springSizeushort	= sizeof(ushort)   * 1 * maxSprings;
    unsigned int springSizefloat    = sizeof(float)    * 1 * maxSprings;
    unsigned int springSizeushort2  = sizeof(ushort)   * 2 * maxSprings;
    unsigned int springSizeuint     = sizeof(uint)     * 1 * maxSprings;
    unsigned int matSizefloat4     	= sizeof(float)    * 4 * COMPOSITE_COUNT;
    unsigned int replaceSizeushort2 = sizeof(ushort)   * 2 * maxReplaced;

	cudaMalloc((void**)&m_dPos, massSizefloat4);
	cudaMalloc((void**)&m_dVel, massSizefloat4);

	cudaMalloc((void**)&m_dPairs,  springSizeushort2);
	cudaMalloc((void**)&m_dRandomPairs,  replaceSizeushort2);
	cudaMalloc((void**)&m_dLbars,  springSizefloat);

	cudaMalloc((void**)&m_dMassMatEncodings,	massSizeuint32_t);
	cudaMalloc((void**)&m_dSpringMatEncodings,	springSizeuint32_t);
	cudaMalloc((void**)&m_dSpringMatIds,		springSizeuint8_t);
	cudaMalloc((void**)&m_dCompositeMats_encoding,	matSizefloat4);
	cudaMalloc((void**)&m_dCompositeMats_id,		matSizefloat4);
	
	cudaMalloc((void**)&m_dMaxStressCount, springSizeushort);
	cudaMalloc((void**)&m_dMinStressCount,  springSizeushort);
	cudaMalloc((void**)&m_dMinStressCount_Sorted,  springSizeushort);

	cudaMalloc((void**)&m_dStresses,  springSizefloat);
	cudaMalloc((void**)&m_dSpringIDs,  springSizeuint);
	cudaMalloc((void**)&m_dSpringIDs_Sorted,  springSizeuint);

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

	maxElements = elements.size();
	maxReplaced = m_replacedSpringsPerElement * maxElements;
	massesPerElement = elements[0].masses.size();
	springsPerElement = elements[0].springs.size();
	maxMasses = massesPerElement*maxElements;
	maxSprings = springsPerElement*maxElements;

	_initialize();

	numMasses = 0; numSprings = 0; numElements = 0;
	for(uint i = 0; i < elements.size(); i++) {
		trackers.push_back(AllocateElement(elements[i]));
	}

	/*
	Notes on SM resources:
	thread blocks	: 8
	threads			: 2048
	registers		: 65536
	shared mem		: 49152
	*/

	uint maxSharedMemSize = 49152;
	uint bytesPerMass = sizeof(float3) + sizeof(float3);
	uint bytesPerMaterial = sizeof(float4);
	// uint bytesPerElement = massesPerElement*bytesPerMass;
	// uint elementsPerBlock = min((maxSharedMemSize - (1<<MATERIAL_COUNT)*bytesPerMaterial) / bytesPerElement, numElements);
	uint elementsPerBlock = 1; // must equal 1
	m_massesPerBlock = massesPerElement * elementsPerBlock;
	m_springsPerBlock = springsPerElement * elementsPerBlock;
	m_sharedMemSizeSim = m_massesPerBlock * bytesPerMass + COMPOSITE_COUNT*bytesPerMaterial;
	m_numBlocksSim = (numElements + elementsPerBlock - 1) / elementsPerBlock;

	// printf("%u / %u\n", m_massesPerBlock, (uint) (maxSharedMemSize - COMPOSITE_COUNT*bytesPerMaterial - sizeof(ushort)*2048) / (bytesPerMass));
	// printf("%u / %u\n", m_sharedMemSizeSim, maxSharedMemSize);

	assert(m_sharedMemSizeSim <= maxSharedMemSize);

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
		float    lbar     	 = springBuf[i].mean_length;
		ushort	 left     	 = springBuf[i].m0,
			     right	  	 = springBuf[i].m1;
		uint8_t	 matId 		 = springBuf[i].material.id;
		uint32_t matEncoding = springBuf[i].material.encoding;

		m_hSpringIDs[i] = i;
		m_hPairs[2*i]   = left;
		m_hPairs[2*i+1] = right;
		m_hLbars[i] 	= lbar;

		m_hSpringMatIds[i] = matId;
		m_hSpringMatEncodings[i] = matEncoding;
	}

	cudaMemcpy(m_dVel, m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dPos, m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	
	cudaMemcpy(m_dMassMatEncodings,		m_hMassMatEncodings,   	 numMasses  * sizeof(uint32_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringMatEncodings,		m_hSpringMatEncodings,   	 numSprings  * sizeof(uint32_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dSpringMatIds,   m_hSpringMatIds,   numSprings * sizeof(uint8_t), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dCompositeMats_id,   m_hCompositeMats_id,   COMPOSITE_COUNT*4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dCompositeMats_encoding,   m_hCompositeMats_encoding,   COMPOSITE_COUNT*4*sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(m_dPairs,  m_hPairs,  numSprings *2*sizeof(ushort),  cudaMemcpyHostToDevice);
	cudaMemcpy(m_dLbars,  m_hLbars,  numSprings  * sizeof(float), cudaMemcpyHostToDevice);

	#ifdef FULL_STRESS
	cudaMemcpy(m_dStresses,   m_hStresses,   numSprings*sizeof(float), cudaMemcpyHostToDevice);
	#endif

	cudaMemcpy(m_dSpringIDs,   m_hSpringIDs,   numSprings*sizeof(uint), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMaxStressCount,   m_hMaxStressCount,   numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dMinStressCount,    m_hMinStressCount,    numSprings*sizeof(ushort), cudaMemcpyHostToDevice);
	gpuErrchk( cudaPeekAtLastError() );

	return trackers;
}

void Simulator::Simulate(float sim_duration, bool trackStresses, bool trace, std::string tracefile) {
	float simTimeRemaining = sim_duration;
	
	short shiftskip = 20;

	SimOptions opt = {
		m_deltaT,
		m_massesPerBlock, m_springsPerBlock,
		numMasses, numSprings,
		COMPOSITE_COUNT,
		shiftskip,
		envBuf[0]
	};
	
	uint step_count = 0;

	static std::map<std::string, int> trace_hist;
	if(trace) {
		if(!trace_hist[tracefile]) trace_hist[tracefile] = 0;
	}

	std::vector<std::tuple<unsigned int, float, float, float, float, float, float, float>> massTrace;
	
	while(simTimeRemaining > 0.0f) {
		if(trackStresses) {
			integrateBodiesStresses<<<m_numBlocksSim,simThreadsPerBlock,m_sharedMemSizeSim>>>(
				(float4*) m_dPos, (float4*) m_dVel,
				(ushort2*)  m_dPairs, (uint32_t*) m_dSpringMatEncodings, (uint8_t*) m_dSpringMatIds,  (float*) m_dLbars,
				(ushort*) m_dMaxStressCount, (ushort*) m_dMinStressCount,
				(float*) m_dStresses, (uint*) m_dSpringIDs, (float4*) m_dCompositeMats_id,
				m_total_time, step_count, opt);
			gpuErrchk( cudaPeekAtLastError() );
			cudaDeviceSynchronize();
		} else {
			integrateBodies<<<m_numBlocksSim,simThreadsPerBlock,m_sharedMemSizeSim>>>(
				(float4*) m_dPos, (float4*) m_dVel,
				(ushort2*)  m_dPairs, (uint8_t*) m_dSpringMatIds,  (float*) m_dLbars,
				(float4*) m_dCompositeMats_id,
				m_total_time, step_count, opt);
			gpuErrchk( cudaPeekAtLastError() );
			cudaDeviceSynchronize();
		}

		if(trace) {
			if(step_count % 20 == 0) {
				cudaMemcpy(m_hPos,m_dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
				cudaMemcpy(m_hVel,m_dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);

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
	tracker.offset_begin = offsetBuf + numSprings;
	tracker.mass_end = tracker.mass_begin; 
	tracker.spring_end = tracker.spring_begin;
	tracker.offset_end = tracker.offset_begin;
	uint massOffset = massesPerElement * numElements;
	
	for(const Mass& m : e.masses) {
		massBuf[numMasses] = m;
		tracker.mass_end++;
		numMasses++;
	}

	uint i = 0;

	// unsigned seed = rand();
	// std::vector<Spring> shuffledSprings(e.springs);
	// std::shuffle(shuffledSprings.begin(), shuffledSprings.end(), std::default_random_engine(seed));

	// for(const Spring& s : shuffledSprings) {
	for(const Spring& s : e.springs) {
		springBuf[numSprings] = s;
		tracker.spring_end++;

		offsetBuf[numSprings] = massOffset;
		tracker.offset_end++;
		numSprings++;
		i++;
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
	cudaMemcpy(m_hPos,m_dPos,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hVel,m_dVel,numMasses*4*sizeof(float),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hMaxStressCount,m_dMaxStressCount,numSprings*sizeof(ushort),cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hMinStressCount, m_dMinStressCount, numSprings*sizeof(ushort),cudaMemcpyDeviceToHost);
	#ifdef FULL_STRESS
	cudaMemcpy(m_hStresses,   m_dStresses,   numSprings*sizeof(float), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hSpringIDs,   m_dSpringIDs,   numSprings*sizeof(uint), cudaMemcpyDeviceToHost);
	#endif

	cudaMemcpy(m_dPos, m_hPos,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(m_dVel, m_hVel,   numMasses   *4*sizeof(float), cudaMemcpyHostToDevice);

	cudaMemcpy(m_hSpringMatEncodings, m_dSpringMatEncodings, numSprings*sizeof(uint32_t), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hPairs, m_dPairs, numSprings*2*sizeof(ushort), cudaMemcpyDeviceToHost);
	cudaMemcpy(m_hLbars, m_dLbars, numSprings * sizeof(float),  cudaMemcpyDeviceToHost);

	for(uint i = 0; i < numSprings; i++) {
		springBuf[i].m0 = m_hPairs[2*i];
		springBuf[i].m1 = m_hPairs[2*i+1];
		springBuf[i].mean_length = m_hLbars[i];
		springBuf[i].material = materials::decode(m_hSpringMatEncodings[i]);
	}

	for(uint i = 0; i < numMasses; i++) {
		float3 pos = {m_hPos[4*i], m_hPos[4*i+1], m_hPos[4*i+2]};
		float3 vel = {m_hVel[4*i], m_hVel[4*i+1], m_hVel[4*i+2]};
		assert(!isnan(pos.x) && !isnan(pos.y) && !isnan(pos.z));

		massBuf[i].pos = Eigen::Vector3f(pos.x,pos.y,pos.z);
		massBuf[i].vel = Eigen::Vector3f(vel.x,vel.y,vel.z);
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
	uint numReplacedSprings = m_replacedSpringsPerElement * numElements;
	uint numBlocks = (numReplacedSprings+threadsPerBlock - 1) / threadsPerBlock;
	randomIntegerPairKernel<<<numBlocks, threadsPerBlock>>>(numReplacedSprings, (ushort2*) m_dRandomPairs,0,massesPerElement-1,seed);

    key_value_sort(m_dMinStressCount, m_dMinStressCount_Sorted, m_dSpringIDs, m_dSpringIDs_Sorted, springsPerElement, numElements);
	// printMinStressCount<<<1, threadsPerBlock>>>(threadsPerBlock*3, m_dMinStressCount_Sorted, m_dSpringIDs_Sorted);

	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );

	uint bytesPerMaterial = sizeof(float4);
	uint sharedMemSize = COMPOSITE_COUNT*bytesPerMaterial;
	numBlocks = (numReplacedSprings + threadsPerBlock - 1) / threadsPerBlock;
	
	DevoOptions opt = {
		numReplacedSprings,
		numSprings,
		springsPerElement,
		massesPerElement,
		m_replacedSpringsPerElement,
		COMPOSITE_COUNT
	};
			
	replaceSprings<<<numBlocks, devoThreadsPerBlock, sharedMemSize>>>(
			(ushort2*) m_dPairs, (uint32_t*) m_dMassMatEncodings,
			(float4*) m_dPos, (float*) m_dLbars,
			(uint32_t*) m_dSpringMatEncodings,
			(uint*) m_dSpringIDs_Sorted, (ushort2*) m_dRandomPairs, 
			(float4*) m_dCompositeMats_encoding, m_total_time,
			opt
		);
	cudaDeviceSynchronize();
	gpuErrchk( cudaPeekAtLastError() );


	cudaMemset(m_dMinStressCount, 0, numSprings * sizeof(ushort));
	cudaMemset(m_dMaxStressCount, 0, numSprings * sizeof(ushort));
	
	seed++;
}

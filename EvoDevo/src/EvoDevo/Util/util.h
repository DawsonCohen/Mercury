#ifndef __UTIL_H__
#define __UTIL_H__

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include "EvoDevo/Core/config.h"

namespace EvoDevo {
    class FileWriter {
    public:
        FileWriter() = default;
        FileWriter(const std::string& filename, bool append = false) { Open(filename, append); }
        ~FileWriter() { Close(); }

        void Open(const std::string& filename, bool append = false);
        void Close() { m_OutputStream.close(); }

        template<typename T>
        void Write(const T& data) {
            m_OutputStream << data;
            m_OutputStream.flush();
        }

        template<typename T>
        friend FileWriter& operator<<(FileWriter& fileWriter, const T& data) {
            fileWriter.Write(data);
            return fileWriter;
        }

    private:
        std::ofstream m_OutputStream;
    };

namespace Util {
    int MakeDirectory(const std::string& directory);


    int WriteFile(const std::string& filename, const std::string& datastring, bool append = false);

    RobotType ReadRobotType(const std::string& filename);

    Config ReadConfigFile(const std::string& filename);

    std::string FitnessHistoryToCSV(std::vector<std::tuple<ulong,float>>& h);

    std::string PopulationFitnessHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    std::string PopulationDiversityHistoryToCSV(std::vector<std::tuple<ulong, std::vector<float>, std::vector<float>>> h);

    Config ReadConfigFile(const std::string& filename);    
}

}

#endif
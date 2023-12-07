#include "EvoDevo/Core/Log.h"

#include "spdlog/sinks/stdout_color_sinks.h"

namespace EvoDevo {

	std::shared_ptr<spdlog::logger> Log::s_CoreLogger = []() {
		spdlog::set_pattern("%^[%T] %n: %v%$");
		auto logger = spdlog::stdout_color_mt("EvoDevo");
		logger->set_level(spdlog::level::trace);
		return logger;
	}();

	std::shared_ptr<spdlog::logger> Log::s_ClientLogger = []() {
		auto logger = spdlog::stdout_color_mt("APP");
		logger->set_level(spdlog::level::trace);
		return logger;
	}();

}
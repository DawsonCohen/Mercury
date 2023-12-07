#pragma once

#include "EvoDevo/Core/Base.h"

#include "spdlog/spdlog.h"
#include <spdlog/fmt/ostr.h>

namespace EvoDevo {

	class Log
	{
	public:
		static void Init();

		inline static std::shared_ptr<spdlog::logger>& GetCoreLogger() { return s_CoreLogger; }
		inline static std::shared_ptr<spdlog::logger>& GetClientLogger() { return s_ClientLogger; }
	private:
		static std::shared_ptr<spdlog::logger> s_CoreLogger;
		static std::shared_ptr<spdlog::logger> s_ClientLogger;
	};


}

// Core log macros
#define EV_CORE_TRACE(...)    ::EvoDevo::Log::GetCoreLogger()->trace(__VA_ARGS__)
#define EV_CORE_INFO(...)     ::EvoDevo::Log::GetCoreLogger()->info(__VA_ARGS__)
#define EV_CORE_WARN(...)     ::EvoDevo::Log::GetCoreLogger()->warn(__VA_ARGS__)
#define EV_CORE_ERROR(...)    ::EvoDevo::Log::GetCoreLogger()->error(__VA_ARGS__)
#define EV_CORE_CRITICAL(...) ::EvoDevo::Log::GetCoreLogger()->critical(__VA_ARGS__)
#define EV_CORE_FATAL(...)    ::EvoDevo::Log::GetCoreLogger()->fatal(__VA_ARGS__)

// Client log macros
#define EV_TRACE(...)	      ::EvoDevo::Log::GetClientLogger()->trace(__VA_ARGS__)
#define EV_INFO(...)	      ::EvoDevo::Log::GetClientLogger()->info(__VA_ARGS__)
#define EV_WARN(...)	      ::EvoDevo::Log::GetClientLogger()->warn(__VA_ARGS__)
#define EV_ERROR(...)	      ::EvoDevo::Log::GetClientLogger()->error(__VA_ARGS__)
#define EV_CRITICAL(...)      ::EvoDevo::Log::GetClientLogger()->critical(__VA_ARGS__)
#define EV_FATAL(...)	      ::EvoDevo::Log::GetClientLogger()->fatal(__VA_ARGS__)
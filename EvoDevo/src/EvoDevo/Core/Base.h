#pragma once

#include <memory>

#include "EvoDevo/Core/Log.h"

#ifdef EV_DEBUG
    #include <signal.h>
    #define EV_DEBUGBREAK() raise(SIGTRAP)

	#define EV_ENABLE_ASSERTS
#endif

#define EV_EXPAND_MACRO(x) x
#define EV_STRINGIFY_MACRO(x) #x

#define BIT(x) (1 << x)
#define EV_BIND_EVENT_FN(fn) [this](auto&&... args) -> decltype(auto) { return this->fn(std::forward<decltype(args)>(args)...); }

namespace EvoDevo {

	template<typename T>
	using Scope = std::unique_ptr<T>;
	template<typename T, typename ... Args>
	constexpr Scope<T> CreateScope(Args&& ... args)
	{
		return std::make_unique<T>(std::forward<Args>(args)...);
	}

	template<typename T>
	using Ref = std::shared_ptr<T>;
	template<typename T, typename ... Args>
	constexpr Ref<T> CreateRef(Args&& ... args)
	{
		return std::make_shared<T>(std::forward<Args>(args)...);
	}

}

#include "EvoDevo/Core/Log.h"
#include "EvoDevo/Core/Assert.h"
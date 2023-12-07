#pragma once

#include "EvoDevo/Core/Base.h"
#include "EvoDevo/Core/Log.h"
#include <filesystem>

#ifdef EV_ENABLE_ASSERTS

	// Alteratively we could use the same "default" message for both "WITH_MSG" and "NO_MSG" and
	// provide support for custom formatting by concatenating the formatting string instead of having the format inside the default message
	#define EV_INTERNAL_ASSERT_IMPL(type, check, msg, ...) { if(!(check)) { EL##type##ERROR(msg, __VA_ARGS__); EV_DEBUGBREAK(); } }
	#define EV_INTERNAL_ASSERT_WITH_MSG(type, check, ...) EV_INTERNAL_ASSERT_IMPL(type, check, "Assertion failed: {0}", __VA_ARGS__)
	#define EV_INTERNAL_ASSERT_NO_MSG(type, check) EV_INTERNAL_ASSERT_IMPL(type, check, "Assertion '{0}' failed at {1}:{2}", EV_STRINGIFY_MACRO(check), std::filesystem::path(__FILE__).filename().string(), __LINE__)

	#define EV_INTERNAL_ASSERT_GET_MACRO_NAME(arg1, arg2, macro, ...) macro
	#define EV_INTERNAL_ASSERT_GET_MACRO(...) EV_EXPAND_MACRO( EV_INTERNAL_ASSERT_GET_MACRO_NAME(__VA_ARGS__, EV_INTERNAL_ASSERT_WITH_MSG, EV_INTERNAL_ASSERT_NO_MSG) )

	// Currently accepts at least the condition and one additional parameter (the message) being optional
	#define EV_ASSERT(...) EV_EXPAND_MACRO( EV_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_, __VA_ARGS__) )
	#define EV_CORE_ASSERT(...) EV_EXPAND_MACRO( EV_INTERNAL_ASSERT_GET_MACRO(__VA_ARGS__)(_CORE_, __VA_ARGS__) )
#else
	#define EV_ASSERT(...)
	#define EV_CORE_ASSERT(...)
#endif

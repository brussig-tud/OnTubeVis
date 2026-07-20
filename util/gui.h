#pragma once

enum class ActionType {
	kUndefined = 0,
	kConfigurationChange,
	kConfigurationValueChange,
	kMappingValueChange,
	kEditRequest
};

// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <carbon/common/Log.h>
#include <status/Provider.h>
#include <ScopedTimer.h>
#include <carbon/common/External.h>

#define RESET_ERROR if(!sc::StatusProvider::isOk()) sc::StatusProvider::set({ 0, "" })
#define SET_ERROR(errorCode,message) sc::StatusProvider::set({ errorCode, message })

#define CHECK_OR_RETURN(condition, returnValue, ...) if (!(condition)) { \
SET_ERROR(-1, epic::carbon::fmt::format(__VA_ARGS__).c_str());\
LOG_ERROR(__VA_ARGS__); return returnValue;\
}

#define HANDLE_EXCEPTION(...) { \
SET_ERROR(-1, epic::carbon::fmt::format(__VA_ARGS__).c_str());\
LOG_ERROR(__VA_ARGS__); return false;\
}

#define PROFILE_FUNC epic::core::ScopedTimer scopedTimer(__func__, LOGGER)

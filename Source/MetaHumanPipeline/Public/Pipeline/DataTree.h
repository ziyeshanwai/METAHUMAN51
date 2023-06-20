// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Misc/ScopeLock.h"
#include "Pipeline/DataTreeTypes.h"
#include "Pipeline/Pin.h"

namespace UE::MetaHuman::Pipeline
{

class METAHUMANPIPELINE_API FDataTree
{
public:

	template<typename T>
	void SetData(const FPin& InPin, const T& InValue)
	{
		checkf(InPin.Direction == EPinDirection::Output, TEXT("Not an output pin"));
		SetData<T>(InPin.Address, InValue);
	}

	template<typename T>
	void SetData(const FPin& InPin, T&& InValue)
	{
		checkf(InPin.Direction == EPinDirection::Output, TEXT("Not an output pin"));
		SetData<T>(InPin.Address, MoveTemp(InValue));
	}

	template<typename T>
	void SetData(const FString& InName, const T& InValue)
	{
		checkf(!InName.IsEmpty(), TEXT("Empty name"));

		FScopeLock ScopeLock(&Mutex);

		TSharedPtr<FDataTree> Child = GetChildOrCreate(InName);

		Child->Data.Set<T>(InValue);
	}

	template<typename T>
	void SetData(const FString& InName, T&& InValue)
	{
		checkf(!InName.IsEmpty(), TEXT("Empty name"));

		FScopeLock ScopeLock(&Mutex);

		TSharedPtr<FDataTree> Child = GetChildOrCreate(InName);

		Child->Data.Set<T>(MoveTemp(InValue));
	}

	template<typename T>
	bool HasData(const FString& InName) const
	{
		checkf(!InName.IsEmpty(), TEXT("Empty name"));

		FScopeLock ScopeLock(&Mutex);

		TSharedPtr<const FDataTree> Child = GetChild(InName);

		return (Child && Child->Data.IsType<T>());
	}

	template<typename T>
	const T& GetData(const FPin& InPin) const
	{		
		checkf(InPin.Direction == EPinDirection::Input, TEXT("Not an input pin"));
		return GetData<T>(InPin.Address);
	}

	template<typename T>
	const T& GetData(const FString& InName) const
	{
		checkf(!InName.IsEmpty(), TEXT("Empty name"));

		FScopeLock ScopeLock(&Mutex);

		TSharedPtr<const FDataTree> Child = GetChild(InName);
		checkf(Child, TEXT("Data does not exist"));
		checkf(Child->Data.IsType<T>(), TEXT("Data is wrong type"));

		return Child->Data.Get<T>();
	}

	template<typename T>
	T&& MoveData(const FString& InName)
	{
		checkf(!InName.IsEmpty(), TEXT("Empty name"));

		FScopeLock ScopeLock(&Mutex);

		TSharedPtr<FDataTree> Child = GetChild(InName);
		checkf(Child, TEXT("Data does not exist"));
		checkf(Child->Data.IsType<T>(), TEXT("Data is wrong type"));

		return MoveTemp(Child->Data.Get<T>());
	}

	void MoveTo(TSharedPtr<FDataTree> InOther);

	FString ToString(int32 InDepth = 0) const;

private:

	FDataTreeType Data;
	TMap<FString, TSharedPtr<FDataTree>> Children;

	mutable FCriticalSection Mutex;

	TSharedPtr<FDataTree> GetChild(const FString& InName) const;
	TSharedPtr<FDataTree> GetChildOrCreate(const FString& InName);
};

}

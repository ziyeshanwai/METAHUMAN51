// Copyright Epic Games, Inc. All Rights Reserved.

#include "AutoRigService.h"
#include "AutoRigServiceBinding.h"

// editor
#include "LevelEditor.h"
#include "ToolMenus.h"

// web browser
#include "HttpModule.h"
#include "SWebBrowser.h"
#include "WebBrowserModule.h"
#include "IWebBrowserSingleton.h"
#include "IWebBrowserCookieManager.h"
#include "JsonDomBuilder.h"
#include "Interfaces/IHttpResponse.h"
#include "Serialization/BufferArchive.h"
#include "Misc/Base64.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/Input/SButton.h"
#include "Misc/FileHelper.h"
#include "GenericPlatform/GenericPlatformFile.h"

#include "MetaHumanEulaDialog.h"
#include "HAL/PlatformFileManager.h"

#define LOCTEXT_NAMESPACE "FAutoRigServiceModule"

namespace
{
	TAutoConsoleVariable<FString> CVarServiceUrl{
		TEXT("mh.ArsServiceUrl"),
		TEXT("https://metahuman-ars.ucs.on.epicgames.com"),
		TEXT("AutoRig Service endpoint"),
		ECVF_Default
	};

	TAutoConsoleVariable<FString> CVarAuthUrl{
		TEXT("mh.ArsServiceAuthUrl"),
		TEXT("https://www.epicgames.com"),
		TEXT("AutoRig Epic auth service endpoint"),
		ECVF_Default
	};

	TAutoConsoleVariable<FString> CVarAuthClientId{
		TEXT("mh.ArsServiceAuthClientId"),
		TEXT("xyza7891OORp4qeFMsqG8MGwJLsun9Tb"),
		TEXT("AutoRig Epic auth service client id"),
		ECVF_Default
	};

	constexpr const TCHAR* MhSessionCookieName = TEXT("mh_ars_session_id.txt");
	constexpr const TCHAR* MhPluginSaveDirectory = TEXT("MhUePlugIn");
	
	enum class ServiceRequest
	{
		SessionInfo = 0,
		Eula_Accept
	};
	
	const FString ServiceEndpoints[] =
	{
		TEXT("/sessionInfo"),
		TEXT("/eula/accept"),
	};

	void ClearBrowserCookies()
	{
		if (const IWebBrowserSingleton* WebBrowserSingleton = IWebBrowserModule::Get().GetSingleton())
		{
			const TSharedPtr<IWebBrowserCookieManager> CookieManager = WebBrowserSingleton->GetCookieManager();
			if (CookieManager.IsValid())
			{
				CookieManager->DeleteCookies();
			}
		}
	}
	
	DECLARE_DELEGATE_OneParam(FServiceRequestCompletedDelegate, FHttpResponsePtr /* Response */);
	
	struct FAutoRigServiceModule : IModuleInterface
	{
		TArray<uint8>							TargetDataX;
		TArray<uint8>							TargetDataY;
		TArray<uint8>							TargetDataZ;
		AutoRigService::TargetSolveParameters	SolveParameters;
		FString									SessionId;
		AutoRigService::FAutoRigServiceLoginCompleteDelegate OnLoginCompleteDelegate;
		AutoRigService::FAutoRigServiceSolveCompleteDelegate OnSolveCompleteDelegate;
		AutoRigService::FAutoRigServiceSolveFailureDelegate OnSolveFailureDelegate;
		bool bLoginWindowClosedByCode;

		// maps a body type index (/2) to a size attribute
		const int32 BodyTypeIndexToSizeAttribute[3] = { 2, 0, 1 };

		/** IModuleInterface implementation */
		virtual void StartupModule() override;
		virtual void ShutdownModule() override;

		void DoLoginRequest()
		{
			IWebBrowserModule::Get();
			const FString AuthUrl = CVarAuthUrl.GetValueOnAnyThread() + TEXT("/id/login?client_id=")
				+ CVarAuthClientId.GetValueOnAnyThread() + TEXT("&response_type=code&redirect_uri=")
			+ CVarServiceUrl.GetValueOnAnyThread() + TEXT("/redirect-login"); 

			// clear this before we attempt to log in so that we are left in a recoverable state if something fails
			SessionId.Empty();
			
			bLoginWindowClosedByCode = false;

			const TSharedRef<SWebBrowser> WebBrowserRef = SAssignNew(TheBrowser, SWebBrowser)
				.InitialURL(AuthUrl)
				.ShowControls(false)
				.OnBeforePopup_Lambda([this](FString NextUrl, FString Target)
					{
						TheBrowser->LoadURL(NextUrl);
						return true;
					})
				.OnUrlChanged_Lambda([this](const FText& Url)
					{
						const FString RedirectedUrl = Url.ToString();
						if (RedirectedUrl.StartsWith(CVarServiceUrl.GetValueOnAnyThread()))
						{
							bLoginWindowClosedByCode = true;
							BrowserWindow.Get()->RequestDestroyWindow();
							TheBrowser.Reset();
						}
					})
			.OnLoadError_Lambda([this]()
			{
				SessionId.Empty();
				UE_LOG(LogTemp, Warning, TEXT("OnLoadError"));
			});

			check(TheBrowser.IsValid());
			{
				if ( BrowserBinding==nullptr )
				{
					BrowserBinding = NewObject<UAutorigServiceBrowserBinding>();
					BrowserBinding->SetBindCallback(UAutorigServiceBrowserBinding::FOnAutoRigServiceBindSessionDelegate::CreateLambda(
						[this]()
					{
							// save a "cookie" with the session ID to provide some session stickiness
							// this cookie contains no sensitive data, it's simply a session handle which may or may not
							// be valid between editor restarts. 
							SessionId = BrowserBinding->GetSessionId();
							FString CookiePath = FPaths::Combine(FPaths::ProjectSavedDir(), MhPluginSaveDirectory);
							IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
							if (!PlatformFile.DirectoryExists(*CookiePath)) 
							{
								PlatformFile.CreateDirectory(*CookiePath);
							}
							CookiePath = FPaths::Combine(CookiePath, MhSessionCookieName);
							FFileHelper::SaveStringToFile(SessionId, *CookiePath,
								FFileHelper::EEncodingOptions::ForceAnsi);
							// checks if we're good, and if we need to accept the EULA for example, then delegates
							// (ultimately) to OnRequestCompleteDelegate
							DoServiceRequest(ServiceRequest::SessionInfo, {});
					}));
				}
				// this binding allows us to call back from JS embedded in the returned HTML document, for example:
				// (js)
				//		window.ue.autorigservicebinding.loginsessionid("SOME_SESSION_ID")
				//
				// (note lowercase!)
				// the snippet above will then invoke UAutorigServiceBrowserBinding::LoginSessionId with "SOME_SESSION_ID"
				//
				TheBrowser->BindUObject(TEXT("AutoRigServiceBinding"), BrowserBinding, true);
			}

			BrowserWindow = SNew(SWindow)
				.Title(FText::FromString(TEXT("Login")))
				.ClientSize(FVector2D(450, 700))
				.SupportsMaximize(false)
				.SupportsMinimize(false)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot()
					.HAlign(HAlign_Fill)
					.VAlign(VAlign_Fill)
					[
						WebBrowserRef
					]
				];

			BrowserWindow.Get()->SetOnWindowClosed(FOnWindowClosed::CreateLambda([this](const TSharedRef<SWindow>& Window)
				{
					// If the user closed the window, the log-in has not succeeded.
					if (!bLoginWindowClosedByCode && SessionId.IsEmpty())
					{
						UE_LOG(LogTemp, Warning, TEXT("Login window closed"));
						ClearBrowserCookies();

						(void)OnSolveFailureDelegate.ExecuteIfBound(AutoRigService::ESolveRequestResult::LoginFailed);
					}
				}));

			FSlateApplication::Get().AddWindow(BrowserWindow.ToSharedRef());
		}


		static TSharedRef<IHttpRequest> CreateHttpRequest()
		{
			const TSharedRef<IHttpRequest> HttpRequest = FHttpModule::Get().CreateRequest();
			HttpRequest->SetHeader(TEXT("User-Agent"), "X-UnrealEngine-Agent");
			return HttpRequest;
		}

		bool HasSessionId() const
		{
			return !SessionId.IsEmpty();
		}

		void SetParamsForSolve(FJsonDomBuilder::FObject & Payload) const
		{
			FJsonDomBuilder::FObject Params;
			Params.Set(TEXT("name"), SolveParameters.IdentityName);
			Params.Set(TEXT("texture_preset"), 5);
			Params.Set(TEXT("body_height"), SolveParameters.Height);
			Params.Set(TEXT("body_gender"), (SolveParameters.BodyTypeIndex & 1) ^ 1);
			Params.Set(TEXT("body_size"), BodyTypeIndexToSizeAttribute[SolveParameters.BodyTypeIndex/2]);
			Params.Set(TEXT("get_deltas"), false);
			Payload.Set(TEXT("params"), Params);
			Payload.Set(TEXT("session_id"), SessionId);
		}
		
		void HttpErrorReporterHelper(const int32 HttpErrorCode) const
		{
			AutoRigService::ESolveRequestResult Result = AutoRigService::ESolveRequestResult::ServerError;
			switch(const int32 ErrorClass = HttpErrorCode/100)
			{
			case 4:
				{
					switch( HttpErrorCode )
					{
					case EHttpResponseCodes::Denied:
						Result = AutoRigService::ESolveRequestResult::Unauthorized;
						break;
					case EHttpResponseCodes::Forbidden:
						Result = AutoRigService::ESolveRequestResult::EulaNotAccepted;
						break;
					default:
						Result = AutoRigService::ESolveRequestResult::InvalidArguments;
						break;
					}
				}
				break;
			case 5:
				{
					if ( HttpErrorCode == EHttpResponseCodes::ServiceUnavail )
					{
						Result = AutoRigService::ESolveRequestResult::Busy; 
					}
				}
				break;
			default:;
			}
			(void)OnSolveFailureDelegate.ExecuteIfBound(Result);
		};

		FServiceRequestCompletedDelegate OnRequestCompleteDelegate;
		/**
		 * @brief Execute an AutoRig service request with session auth and EULA checks
		 * @param Request The actual request to execute and which will be executed iff the user (is) logged in, and the EULA is accepted.
		 * @param OnRequestCompleteDelegate_ Delegate to call if the request ultimately succeeds
		 *
		 * This implements a basic, and fixed, state machine using delegates which roughly goes through the following steps
		 *
		 *    Is session still valid? (/sessionInfo) -> no -> login (/login) 
		 *             \                                    /
		 *              yes           _____________________/
		 *               \           /
		 *              has user accepted EULA? -> no -> display EULA and require acceptance (/eula/accept)
		 *                 \                                 /
		 *                 yes        ______________________/
		 *                   \       /
		 *                  do service request
		 *                    \
		 *                    succeeded? -> no -> invoke user failure delegate
		 *                     \
		 *                      yes
		 *                       \
		 *                       invoke user success delegate 
		 */
		void DoServiceRequest(ServiceRequest Request, FServiceRequestCompletedDelegate&& OnRequestCompleteDelegate_)
		{
			// Only take this if it's bound. We rely on this later if we chain requests. 
			if ( OnRequestCompleteDelegate_.IsBound() )
			{
				OnRequestCompleteDelegate = MoveTemp(OnRequestCompleteDelegate_);
			}
			const TSharedRef<IHttpRequest> HttpRequest = CreateHttpRequest();
			const FString Url = CVarServiceUrl.GetValueOnAnyThread() +
				TEXT("/autorigservice") +
				ServiceEndpoints[static_cast<int32>(Request)];
			HttpRequest->SetURL(Url);
			HttpRequest->SetVerb("POST");
			HttpRequest->SetHeader("Content-Type", TEXT("application/json"));
			HttpRequest->OnProcessRequestComplete().BindLambda(
				[this, Request](FHttpRequestPtr, FHttpResponsePtr Response, bool bCompletedOk)
				{
					if ( bCompletedOk
						&&
						EHttpResponseCodes::IsOk(Response->GetResponseCode())
					)
					{
						//NOTE: invoked even if the result is empty
						(void)OnRequestCompleteDelegate.ExecuteIfBound(Response);
					}
					else
					{
						if ( Response->GetResponseCode() == EHttpResponseCodes::Denied )
						{
							// we're not logged in, trigger the flow
							// this will re-issue a sessionInfo request later which will trampoline to
							// the request completion delegate bound at this point, completing the loop
							DoLoginRequest();
						}
						else if ( Request == ServiceRequest::SessionInfo
							&&
							Response->GetResponseCode() == EHttpResponseCodes::Forbidden)
						{
							// EULA is not yet accepted, we need to trigger the EULA flow
							const FString ResponseData = Response->GetContentAsString();
							TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject);
							const TSharedRef<TJsonReader<TCHAR>> JsonReader = TJsonReaderFactory<TCHAR>::Create(ResponseData);
							if ( FJsonSerializer::Deserialize<TCHAR>(JsonReader.Get(), JsonObject)
								&&
								JsonObject.IsValid() )
							{
								static const FString SessionIdFieldName = "session_id";
								static const FString EulaCheckFieldName = "eula";
								check(JsonObject->HasField(SessionIdFieldName) && JsonObject->GetField<EJson::String>(SessionIdFieldName));
								if ( JsonObject->HasField(EulaCheckFieldName) && JsonObject->GetField<EJson::String>(EulaCheckFieldName) )
								{
									const TSharedPtr<FJsonValue> Eula = JsonObject->GetField<EJson::String>(EulaCheckFieldName);
									const FVector2D EulaWindowSize = FVector2D(1008, 566);

									const TSharedPtr<SWindow> EulaWindow =
										SNew(SWindow)
										.Title(LOCTEXT("MHEula_Title", "End User License Agreement"))
										.ClientSize(EulaWindowSize)
										.MinHeight(EulaWindowSize.Y)
										.MinWidth(EulaWindowSize.X)
										.SizingRule(ESizingRule::UserSized)
										.SupportsMinimize(false)
										.SupportsMaximize(false);

									const TSharedRef<SMetaHumanEulaDialog> EulaDialog =
										SNew(SMetaHumanEulaDialog)
										.ParentWindow(EulaWindow)
										.EulaText(FText::FromString(Eula->AsString()));

									EulaWindow->SetContent(EulaDialog);
									FSlateApplication::Get().AddModalWindow(EulaWindow.ToSharedRef(), FGlobalTabmanager::Get()->GetRootWindow());

									if (EulaDialog->WasEulaAccepted())
									{
										UE_LOG(LogTemp, Warning, TEXT("accepted EULA"));
										DoServiceRequest(ServiceRequest::Eula_Accept, {});
									}
									else
									{
										// the server will continue to refuse service until this is accepted, we don't have to do anything here  
										(void)OnSolveFailureDelegate.ExecuteIfBound(AutoRigService::ESolveRequestResult::EulaNotAccepted);
									}
								}
							}
						}
						else
						{
							HttpErrorReporterHelper(Response->GetResponseCode());
						}
					}
				});
			
			FJsonDomBuilder::FObject PayloadObject;
			PayloadObject.Set(TEXT("session_id"), SessionId);
			HttpRequest->SetContentAsString(PayloadObject.ToString());
			HttpRequest->ProcessRequest();
		}
		
		void DoSolveImplementation()
		{
			if ( !HasSessionId() )
			{
				OnLoginCompleteDelegate = AutoRigService::FAutoRigServiceLoginCompleteDelegate::CreateLambda(
					[this]()->void
				{
					DoSolveImplementation();
				});
				DoLoginRequest();
				return;
			}
			
			DoServiceRequest(ServiceRequest::SessionInfo,
				FServiceRequestCompletedDelegate::CreateLambda([this](FHttpResponsePtr)
			{
				// NOTE: there is a bug in the CurlHttp implementation which prevents us from re-using a Request object
				//		 TL;DR the second request gets truncated to whatever the length of the first one was.
				//		 See: https://jira.it.epicgames.com/browse/UE-140135
				//
				const TSharedRef<IHttpRequest> HttpRequest = CreateHttpRequest();
				HttpRequest->OnProcessRequestComplete().BindLambda(
					[this](FHttpRequestPtr Request, FHttpResponsePtr Response, bool bCompletedOk)
				{
						(void)Request;
						if ( bCompletedOk
							&&
							EHttpResponseCodes::IsOk(Response->GetResponseCode())
							)
						{
							//NOTE: invoked even if the result is empty
							OnSolveCompleteDelegate.Execute(Response->GetContent());
						}
						else
						{
							HttpErrorReporterHelper(Response->GetResponseCode());
						}
				});
				
				HttpRequest->SetURL(CVarServiceUrl.GetValueOnAnyThread() + TEXT("/autorigservice/solve4"));
				HttpRequest->SetVerb("POST");
				HttpRequest->SetHeader("Content-Type", TEXT("application/json"));
				FJsonDomBuilder::FObject PayloadObject;
				SetParamsForSolve(PayloadObject);
				PayloadObject.Set(TEXT("target_data_x"), FBase64::Encode(reinterpret_cast<const uint8*>(TargetDataX.GetData()), TargetDataX.Num()));
				PayloadObject.Set(TEXT("target_data_y"), FBase64::Encode(reinterpret_cast<const uint8*>(TargetDataY.GetData()), TargetDataY.Num()));
				PayloadObject.Set(TEXT("target_data_z"), FBase64::Encode(reinterpret_cast<const uint8*>(TargetDataZ.GetData()), TargetDataZ.Num()));
				HttpRequest->SetContentAsString(PayloadObject.ToString());
				HttpRequest->ProcessRequest();
			}));
		}
		
		void DoSolve(TArray<FVector3d>&& InVertices,
			const AutoRigService::TargetSolveParameters & Params,
			AutoRigService::FAutoRigServiceSolveCompleteDelegate&& OnSolveCompletedDelegate_,
			AutoRigService::FAutoRigServiceSolveFailureDelegate&& OnAutoRigServiceSolveFailureDelegate_)
		{
			check(InVertices.Num()>0);
			check(InVertices.Num()<=25000);
			
			// convert the input vertices to a packed format so that we can reliably ship them to the service
			// NOTE: this isn't fast, but compared to uploading an processing it is not time critical
			constexpr auto PushPackFunc = [](const double Q, TArray<uint8>& Target)
			{
				const float Float32Point = static_cast<float>(Q);
				const uint8* RawFloat32 = reinterpret_cast<const uint8*>(&Float32Point);
#ifdef PLATFORM_LITTLE_ENDIAN
				Target.Push(RawFloat32[0]);
				Target.Push(RawFloat32[1]);
				Target.Push(RawFloat32[2]);
				Target.Push(RawFloat32[3]);
#else
				Target.Push(RawFloat32[3]);
				Target.Push(RawFloat32[2]);
				Target.Push(RawFloat32[1]);
				Target.Push(RawFloat32[0]);
#endif
			};
			TargetDataX.Empty();
			TargetDataY.Empty();
			TargetDataZ.Empty();
			for( const auto& Vertex: InVertices)
			{
				PushPackFunc(Vertex.X, TargetDataX);
				PushPackFunc(Vertex.Y, TargetDataY);
				PushPackFunc(Vertex.Z, TargetDataZ);
			}
			OnSolveCompleteDelegate = MoveTemp(OnSolveCompletedDelegate_);
			OnSolveFailureDelegate = MoveTemp(OnAutoRigServiceSolveFailureDelegate_);
			SolveParameters = Params;
			DoSolveImplementation();
		}
		
		void DoLogin(AutoRigService::FAutoRigServiceLoginCompleteDelegate&& OnLoginCompleteDelegate_)
		{
			OnLoginCompleteDelegate = MoveTemp(OnLoginCompleteDelegate_);
			if ( !HasSessionId() )
			{
				// if a session "cookie" is present we might still be able to use it
				FString CookiePath = FPaths::Combine(FPaths::ProjectSavedDir(), MhPluginSaveDirectory);
				IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
				if (PlatformFile.DirectoryExists(*CookiePath)) 
				{
					CookiePath = FPaths::Combine(CookiePath, MhSessionCookieName);
					FFileHelper::LoadFileToString(SessionId, *CookiePath, FFileHelper::EHashOptions::None);
				}
			}
			
			// this will trigger a login if needed
			DoServiceRequest(ServiceRequest::SessionInfo,
				FServiceRequestCompletedDelegate::CreateLambda([this](FHttpResponsePtr)
			{
					(void)OnLoginCompleteDelegate.ExecuteIfBound();
			}));
		}
		
		TSharedPtr<class FUICommandList> PluginCommands;
		TSharedPtr<SWindow> BrowserWindow;
		TSharedPtr<SWebBrowser> TheBrowser;
		static UAutorigServiceBrowserBinding* BrowserBinding;
	};

	UAutorigServiceBrowserBinding* FAutoRigServiceModule::BrowserBinding = nullptr;
	

}

void FAutoRigServiceModule::StartupModule()
{
}

void FAutoRigServiceModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FAutoRigServiceModule, AutoRigService)
namespace AutoRigService
{
	void SolveForTarget(TArray<FVector3d>&& InVertices,
	const TargetSolveParameters& InParams,
	FAutoRigServiceSolveCompleteDelegate&& OnAutoRigServiceSolveCompleteDelegate,
	FAutoRigServiceSolveFailureDelegate&& OnAutoRigServiceSolveFailureDelegate)
	{
		if ( InVertices.IsEmpty() )
		{
			(void)OnAutoRigServiceSolveFailureDelegate.ExecuteIfBound(ESolveRequestResult::InvalidArguments);
		}
		FAutoRigServiceModule& AutoRigServiceModule = FModuleManager::LoadModuleChecked<FAutoRigServiceModule>("AutoRigService");
		AutoRigServiceModule.DoSolve(Forward<TArray<FVector3d>>(InVertices),
			InParams,
			MoveTemp(OnAutoRigServiceSolveCompleteDelegate),
			MoveTemp(OnAutoRigServiceSolveFailureDelegate)
			);
	}
	
	bool IsLoggedIn()
	{
		const FAutoRigServiceModule& AutoRigServiceModule = FModuleManager::LoadModuleChecked<FAutoRigServiceModule>("AutoRigService");
		return AutoRigServiceModule.HasSessionId();
	}
	
	void Login(FAutoRigServiceLoginCompleteDelegate&& OnAutoRigServiceLoginCompleteDelegate)
	{
		FAutoRigServiceModule& AutoRigServiceModule = FModuleManager::LoadModuleChecked<FAutoRigServiceModule>("AutoRigService");
		AutoRigServiceModule.DoLogin(Forward<FAutoRigServiceLoginCompleteDelegate>(OnAutoRigServiceLoginCompleteDelegate));
	}
}
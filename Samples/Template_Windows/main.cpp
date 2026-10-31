#include "WickedEngine.h"


class Game : public wi::Application
{
	wi::RenderPath3D path;
public:
	void Initialize() override
	{
		wi::Application::Initialize();

		ActivatePath(&path);

		path.setMotionBlurEnabled(true);
		path.setMotionBlurStrength(0.35f);
		//path.setMSAASampleCount(1);
		path.setFXAAEnabled(false);
		path.setAO(wi::RenderPath3D::AO_MSAO);
		path.setAOPower(1.0f);
		//path.setAORange(5.0f);
		//path.setAOSampleCount(8);
		path.setEyeAdaptionEnabled(true);
		path.setSharpenFilterEnabled(true);
		path.setSharpenFilterAmount(0.15f);
		path.setTonemap(wi::renderer::Tonemap::Uchimura);
		path.setEyeAdaptionKey(0.2f);   // target brightness; higher = brighter overall
		path.setDitherEnabled(false);
		path.setBloomThreshold(5.0f);
		path.setSSREnabled(true);
		path.setReflectionRoughnessCutoff(0.55f);
		wi::scene::LoadModel("content/DrivePhysx.wiscene");
		wi::lua::RunFile("content/car_physicsv3.lua");
		wi::renderer::SetTemporalAAEnabled(true);
		wi::renderer::SetShadowProps2D(2048);
		wi::eventhandler::SetVSync(false);
	}
};

Game application;   // replaces "wi::Application application;"
int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
	_In_opt_ HINSTANCE hPrevInstance,
	_In_ LPWSTR    lpCmdLine,
	_In_ int       nCmdShow)
{
	// Win32 window and message loop setup:
	static auto WndProc = [](HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) -> LRESULT
	{
		switch (message)
		{
		case WM_SIZE:
		case WM_DPICHANGED:
			if (application.is_window_active)
				application.SetWindow(hWnd);
			break;
		case WM_CHAR:
			switch (wParam)
			{
			case VK_BACK:
				wi::gui::TextInputField::DeleteFromInput();
				break;
			case VK_RETURN:
				break;
			default:
			{
				const wchar_t c = (const wchar_t)wParam;
				wi::gui::TextInputField::AddInput(c);
			}
			break;
			}
			break;
		case WM_INPUT:
			wi::input::rawinput::ParseMessage((void*)lParam);
			break;
		case WM_KILLFOCUS:
			application.is_window_active = false;
			break;
		case WM_SETFOCUS:
			application.is_window_active = true;
			break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return 0;
	};
	SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
	WNDCLASSEXW wcex = {};
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = NULL;
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = NULL;
	wcex.lpszClassName = L"DrivePhysx - Theironlefty's Game";
	wcex.hIconSm = NULL;
	RegisterClassExW(&wcex);
	HWND hWnd = CreateWindowW(wcex.lpszClassName, wcex.lpszClassName, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);
	ShowWindow(hWnd, SW_SHOWDEFAULT);

	// set Win32 window to engine:
	application.SetWindow(hWnd);

	// process command line string:
	wi::arguments::Parse(lpCmdLine);

	// just show some basic info:
	application.infoDisplay.active = true;
	application.infoDisplay.watermark = true;
	application.infoDisplay.resolution = true;
	application.infoDisplay.fpsinfo = true;

	MSG msg = { 0 };
	while (msg.message != WM_QUIT)
	{
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else {

			application.Run();

		}
	}

	wi::jobsystem::ShutDown(); // waits for jobs to finish before shutdown

	return (int)msg.wParam;
}

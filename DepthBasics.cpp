//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include <iostream>
#include <cstdio>

#include "stdafx.h"
#include <strsafe.h>
#include "resource.h"
#include "DepthBasics.h"

#include <zmq.h>
#include <zmq_utils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "tracking.h"

using namespace std;

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(
	_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR lpCmdLine,
    _In_ int nShowCmd
)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    CDepthBasics application;
    application.Run(hInstance, nShowCmd);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_bSaveScreenshot(false),
    m_pKinectSensor(NULL),
    m_pDepthFrameReader(NULL),
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_pDepthRGBX(NULL)
{
    LARGE_INTEGER qpf = {0};
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }

    // create heap storage for depth pixel data in RGBX format
    m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];

	void *context = zmq_init(1);

	m_socket = zmq_socket(context, ZMQ_PUSH);
	//zmq_connect(m_socket, "tcp://172.16.13.58:1213");
	zmq_bind(m_socket, "tcp://0.0.0.0:1213");
}
  

/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics()
{
    // clean up Direct2D renderer
    if (m_pDrawDepth)
    {
        delete m_pDrawDepth;
        m_pDrawDepth = NULL;
    }

    if (m_pDepthRGBX)
    {
        delete [] m_pDepthRGBX;
        m_pDepthRGBX = NULL;
    }

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    // done with depth frame reader
    SafeRelease(m_pDepthFrameReader);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"DepthBasicsAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        NULL,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        Update();

        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if (hWndApp && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update()
{
	static float x=0, y=0, z=0, angle=0;
	static char json[200];
	static int detect_x = 200, detect_y = 150;
	static bool detected = false;
	static int window = -1;

    if (!m_pDepthFrameReader)
    {
        return;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;
        IFrameDescription* pFrameDescription = NULL;
        int nWidth = 0;
        int nHeight = 0;
        USHORT nDepthMinReliableDistance = 0;
        USHORT nDepthMaxDistance = 0;
        UINT nBufferSize = 0;
        UINT16 *pBuffer = NULL;

        pDepthFrame->get_RelativeTime(&nTime);  
		pDepthFrame->get_FrameDescription(&pFrameDescription);
        pFrameDescription->get_Width(&nWidth);
		pFrameDescription->get_Height(&nHeight);
        pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);

		// In order to see the full range of depth (including the less reliable far field depth)
		// we are setting nDepthMaxDistance to the extreme potential depth threshold
		nDepthMaxDistance = USHRT_MAX;

		// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
        pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		
		//pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);            
        
		//ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);

		UINT16 * depthBuffer = new UINT16[nHeight * nWidth];
		pDepthFrame->CopyFrameDataToArray(nHeight * nWidth, depthBuffer);
		cv::Mat depthMap = cv::Mat(nHeight, nWidth, CV_16U, depthBuffer);
		cv::Mat fDepthMap = cv::Mat(nHeight, nWidth, CV_32FC1);
		depthMap.convertTo(fDepthMap, CV_32FC1);

		SafeRelease(pFrameDescription);

		// Get infrared frame
		pFrameDescription = NULL;
		IInfraredFrame* pInfraredFrame = NULL;
		m_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);
		if (!pInfraredFrame)  {
			SafeRelease(pFrameDescription);
			SafeRelease(pInfraredFrame);

			SafeRelease(pDepthFrame);
			return;
		}
		pInfraredFrame->get_FrameDescription(&pFrameDescription);
		pFrameDescription->get_Width(&nWidth);
		pFrameDescription->get_Height(&nHeight);
		//pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

		UINT16 * irBuffer = new UINT16[nHeight * nWidth];
		pInfraredFrame->CopyFrameDataToArray(nHeight * nWidth, irBuffer);
		cv::Mat irImage = cv::Mat(nHeight, nWidth, CV_16U, irBuffer);
		cv::Mat fIrImage = cv::Mat(nHeight, nWidth, CV_32FC1);
		irImage.convertTo(fIrImage, CV_32FC1);

		cv::imshow("ir", fIrImage / 20000.0f);
		cv::imshow("depth", fDepthMap / 4500.0f);

		cv::Mat th_display(fIrImage.size(), CV_8UC3, cv::Scalar(255, 255, 255));

		cv::Rect roi;
		if (detected) window = 100;
		else if (window > 0) {
			window += 25;
			if (window > 300) window = -1;
		}

		if (detected || window > 0) {
			int x = detect_x-window/2, y = detect_y-window/2;

			x = max(min(x, fIrImage.size().width - (window+1)), 0);
			y = max(min(y, fIrImage.size().height - (window+1)), 0);

			roi = cv::Rect(x, y, window, window);

			cv::rectangle(th_display, roi, cv::Scalar(1.0, 0.0, 0.0));
		} else
			roi = cv::Rect(0, 0, fIrImage.size().width, fIrImage.size().height);

		cv::Mat ir_roi(fIrImage, roi);
		cv::Mat th_roi(th_display, roi);

		cf_instance cf;
		detect_cfs(&ir_roi, &fDepthMap, &th_roi, &cf);

		if (cf.found) {
			cf.x += roi.x;
			cf.y += roi.y;
			cf.x_d += roi.x;
			cf.y_d += roi.y;
		}

		//circle(th_display, depth_node, radius[pi[tip]], Scalar(150,150,150), 2);
		if (cf.found == true && cf.x_d >= 0 && cf.x_d < 512 && cf.y_d >= 0 && cf.y_d < 424) {

			detected = true;
			detect_x = cf.x;
			detect_y = cf.y;

			DepthSpacePoint cfPoint;
			CameraSpacePoint cfPos;
			//float cfDepth = fDepthMap.at<float>(cf.x_d, cf.y_d);
			UINT16 cfDepth = *(depthBuffer + (int) cf.y_d * 512 + (int) cf.x_d);
			cfPoint.X = cf.x;
			cfPoint.Y = cf.y;

			ICoordinateMapper * mapper;
			m_pKinectSensor->get_CoordinateMapper(&mapper);
			mapper->MapDepthPointToCameraSpace(cfPoint, cfDepth, &cfPos);

			//get_depth_at_xy(static_cast<void*>(depth->data), &cf);
			cout << "CF at " << cfPos.X << "x" << cfPos.Y << "x" << cfPos.Z << endl;
			x = cfPos.X; y = cfPos.Y; z = cfPos.Z; angle = cf.angle;
			sprintf(json, "{\"pos\": [%f, %f, %f], \"angle\": %f, \"detect\": true}", x, y, z, angle);
			zmq_send(m_socket, json, strlen(json), ZMQ_NOBLOCK);

			char coord[200];
			sprintf(coord, "x[%.1f], y[%.1f], z[%f]", cfPos.X*1000, cfPos.Y*1000, cfPos.Z);
			cv::putText(th_display, coord, cv::Point(cf.x_d, cf.y_d), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(100.0, 10.0, 250.0), 1, CV_AA);
			sprintf(coord, "x[%d] y[%d] d[%d]", (int)cf.x, (int)cf.y, cfDepth);
			cv::putText(th_display, coord, cv::Point(cf.x_d, cf.y_d-20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(100.0, 10.0, 250.0), 1, CV_AA);
			sprintf(coord, "A[%d]", (int)cf.angle, (int)cf.y, cfDepth);
			cv::putText(th_display, coord, cv::Point(cf.x_d, cf.y_d - 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(100.0, 10.0, 250.0), 1, CV_AA);
		}
		else {
			detected = false;

			if (!_finite(x)) x = 0;
			if (!_finite(y)) y = 0;
			if (!_finite(z)) z = 0;

			cout << "No CF found :-(" << endl;
			// Send old stuff if nothing was found
			sprintf(json, "{\"pos\": [%f, %f, %f], \"angle\": %f, \"detect\": false}", x, y, z, angle);
			zmq_send(m_socket, json, strlen(json), ZMQ_NOBLOCK);
		}


		imshow("TH display", th_display);


        SafeRelease(pFrameDescription);
		SafeRelease(pInfraredFrame);
		delete(irBuffer);
		delete(depthBuffer);
    }

    SafeRelease(pDepthFrame);
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(wParam);
    UNREFERENCED_PARAMETER(lParam);

    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawDepth = new ImageRenderer();
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(RGBQUAD)); 
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.", 10000, true);
            }

            // Get and initialize the default Kinect sensor
            InitializeDefaultSensor();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the screenshot control and a button clicked event, save a screenshot next frame 
            if (IDC_BUTTON_SCREENSHOT == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                m_bSaveScreenshot = true;
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

		// Init IR
        SafeRelease(pDepthFrameSource);

		IInfraredFrameSource* pInfraredFrameSource = NULL;

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pInfraredFrameSource->OpenReader(&m_pInfraredFrameReader);
		}

		SafeRelease(pInfraredFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!", 10000, true);
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pBuffer">pointer to frame data</param>
/// <param name="nWidth">width (in pixels) of input image data</param>
/// <param name="nHeight">height (in pixels) of input image data</param>
/// <param name="nMinDepth">minimum reliable depth</param>
/// <param name="nMaxDepth">maximum reliable depth</param>
/// </summary>
void CDepthBasics::ProcessDepth(INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
    if (m_hWnd)
    {
        if (!m_nStartTime)
        {
            m_nStartTime = nTime;
        }

        double fps = 0.0;

        LARGE_INTEGER qpcNow = {0};
        if (m_fFreq)
        {
            if (QueryPerformanceCounter(&qpcNow))
            {
                if (m_nLastCounter)
                {
                    m_nFramesSinceUpdate++;
                    fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
                }
            }
        }

        WCHAR szStatusMessage[64];
        StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L" FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

        if (SetStatusMessage(szStatusMessage, 1000, false))
        {
            m_nLastCounter = qpcNow.QuadPart;
            m_nFramesSinceUpdate = 0;
        }
    }

    // Make sure we've received valid data
    if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
    {
        RGBQUAD* pRGBX = m_pDepthRGBX;

        // end pixel is start + width*height - 1
        const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

        while (pBuffer < pBufferEnd)
        {
            USHORT depth = *pBuffer;

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

            pRGBX->rgbRed   = intensity;
            pRGBX->rgbGreen = intensity;
            pRGBX->rgbBlue  = intensity;

            ++pRGBX;
            ++pBuffer;
        }

        // Draw the data with Direct2D
        m_pDrawDepth->Draw(reinterpret_cast<BYTE*>(m_pDepthRGBX), cDepthWidth * cDepthHeight * sizeof(RGBQUAD));

        if (m_bSaveScreenshot)
        {
            WCHAR szScreenshotPath[MAX_PATH];

            // Retrieve the path to My Photos
            GetScreenshotFileName(szScreenshotPath, _countof(szScreenshotPath));

            // Write out the bitmap to disk
            HRESULT hr = SaveBitmapToFile(reinterpret_cast<BYTE*>(m_pDepthRGBX), nWidth, nHeight, sizeof(RGBQUAD) * 8, szScreenshotPath);

            WCHAR szStatusMessage[64 + MAX_PATH];
            if (SUCCEEDED(hr))
            {
                // Set the status bar to show where the screenshot was saved
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Screenshot saved to %s", szScreenshotPath);
            }
            else
            {
                StringCchPrintf(szStatusMessage, _countof(szStatusMessage), L"Failed to write screenshot to %s", szScreenshotPath);
            }

            SetStatusMessage(szStatusMessage, 5000, true);

            // toggle off so we don't save a screenshot again next frame
            m_bSaveScreenshot = false;
        }
    }
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
bool CDepthBasics::SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce)
{
    INT64 now = GetTickCount64();

    if (m_hWnd && (bForce || (m_nNextStatusTime <= now)))
    {
        SetDlgItemText(m_hWnd, IDC_STATUS, szMessage);
        m_nNextStatusTime = now + nShowTimeMsec;

        return true;
    }

    return false;
}

/// <summary>
/// Get the name of the file where screenshot will be stored.
/// </summary>
/// <param name="lpszFilePath">string buffer that will receive screenshot file name.</param>
/// <param name="nFilePathSize">number of characters in lpszFilePath string buffer.</param>
/// <returns>
/// S_OK on success, otherwise failure code.
/// </returns>
HRESULT CDepthBasics::GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize)
{
    WCHAR* pszKnownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);

    if (SUCCEEDED(hr))
    {
        // Get the time
        WCHAR szTimeString[MAX_PATH];
        GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

        // File name will be KinectScreenshotDepth-HH-MM-SS.bmp
        StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-Depth-%s.bmp", pszKnownPath, szTimeString);
    }

    if (pszKnownPath)
    {
        CoTaskMemFree(pszKnownPath);
    }

    return hr;
}

/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
    DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

    BITMAPINFOHEADER bmpInfoHeader = {0};

    bmpInfoHeader.biSize        = sizeof(BITMAPINFOHEADER);  // Size of the header
    bmpInfoHeader.biBitCount    = wBitsPerPixel;             // Bit count
    bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
    bmpInfoHeader.biWidth       = lWidth;                    // Width in pixels
    bmpInfoHeader.biHeight      = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
    bmpInfoHeader.biPlanes      = 1;                         // Default
    bmpInfoHeader.biSizeImage   = dwByteCount;               // Image size in bytes

    BITMAPFILEHEADER bfh = {0};

    bfh.bfType    = 0x4D42;                                           // 'M''B', indicates bitmap
    bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
    bfh.bfSize    = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

    // Create the file on disk to write to
    HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

    // Return if error opening file
    if (NULL == hFile) 
    {
        return E_ACCESSDENIED;
    }

    DWORD dwBytesWritten = 0;
    
    // Write the bitmap file header
    if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the bitmap info header
    if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }
    
    // Write the RGB Data
    if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
    {
        CloseHandle(hFile);
        return E_FAIL;
    }    

    // Close the file
    CloseHandle(hFile);
    return S_OK;
}

/*
* MIT License
*
* Copyright (c) 2020 Robert Hutter
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* AlticraftLogPreprocessor.cpp - Main file of the Alticraft Log Preprocessor.
*   Execution start here.
*
* This program preprocesses a workspace created by the Alticraft Flight
* Data Recorder. This program stiches together multiple logfiles into one.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WORKSPACE_EXTENSION L"\\FDR_LOG.CSV"
#define LOGFILE_PREFIX L"\\LOG_"
#define LOGFILE_SUFFIX L".CSV"

FILE* logf;
FILE* outp;
wchar_t* workspace;

void wgetlogp(wchar_t** logpath, wchar_t* workspace, unsigned int logn);

/**
* Main function with command-line arguments.
* Command-line arguments:
* 1. Source of workspace containing logfiles.
*
*/
int wmain(int argc, wchar_t* argv[])
{
    printf("Alticraft FDR Log Preprocessor - Preprocesses FDR workspace for further data analysis.\n");
    printf("Version: 1.0.0.\n");
    printf("Build date: %s\n", __DATE__);
    printf("Icon credit: Smashicons from flaticon.com\n\n");

    // Insufficient arguements, no path given
    if (argc < 2)
    {
        printf("Error: Insufficiant arguments given.\n\n");
        printf("Usage: [program] [workspace_directory]\n\n");
        printf("To use this software, provide the path of the workspace as the workspace_directory argument.\n");
        printf("Provide workspace directory in double quotes if it contains whitespaces.\n");
        printf("Output: FDR_LOG.CSV - A file containing all data from FDR workspace.\n");
        return EXIT_FAILURE;
    }

    workspace = argv[1];

    // Feedback
    wprintf(L"Workspace selected: %ls\n", workspace);

    // Get path of FDRLog.csv
    unsigned int outpnamelen = wcslen(workspace) + wcslen(WORKSPACE_EXTENSION) + 1;
    wchar_t* outpname = (wchar_t*) malloc(outpnamelen * sizeof(wchar_t));

    wcscpy_s(outpname, outpnamelen, workspace);
    wcscat_s(outpname, outpnamelen, WORKSPACE_EXTENSION);

    // Create file
    if (_wfopen_s(&outp, outpname, L"w") != 0)
    {
        // IO error occured
        printf("Error: IO error occured writing to output logfile.\n\n");
        printf("Make sure output file is accessable and try again.\n");
        return EXIT_FAILURE;
    }

    // Feedback
    wprintf(L"Merged log file created at: %ls\n", outpname);

    unsigned int logn = 0;
    wchar_t* logpath{};
    wgetlogp(&logpath, workspace, logn);

    // Open logfile
    for (int tries = 0; tries < 3; tries++)
    {
        while (_wfopen_s(&logf, logpath, L"r") == 0)
        {
            wprintf(L"Parsing: %ls\n", logpath);
            // Copy data to output file
            char c = fgetc(logf);
            while (c != EOF)
            {
                fwrite(&c, 1, 1, outp);
                c = fgetc(logf);
            }

            // Close logfile and prepair next one
            fclose(logf);
            wgetlogp(&logpath, workspace, ++logn);
            tries = 0;
        }
        wgetlogp(&logpath, workspace, ++logn);
    }
    fclose(outp);

    printf("\nDone!\n");

    return EXIT_SUCCESS;
}

/**
* Returns path of log file with specified number. Allocates sufficiant memory for path,
* meaning reallocation if not nullptr.
*
* @param logpath - Pointer to the variable which will point to the path.
* @param workspace
* @param logn
*/
void wgetlogp(wchar_t** logpath, wchar_t* workspace, unsigned int logn)
{
    wchar_t wlogn[32];
    swprintf_s(wlogn, 32, L"%u", logn);

    if (*logpath != NULL)
        free(*logpath);

    unsigned int logpathlen = wcslen(workspace) + wcslen(LOGFILE_PREFIX) + wcslen(wlogn) + wcslen(LOGFILE_SUFFIX) + 1;
    
    *logpath = (wchar_t*) malloc(logpathlen * sizeof(wchar_t));
    wcscpy_s(*logpath, logpathlen, workspace);
    wcscat_s(*logpath, logpathlen, LOGFILE_PREFIX);
    wcscat_s(*logpath, logpathlen, wlogn);
    wcscat_s(*logpath, logpathlen, LOGFILE_SUFFIX);
}
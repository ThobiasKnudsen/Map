#pragma once

#include <Windows.h>
#include <vector>
#include <string>
#include <chrono>


class DBG {
// structs
private:
    struct FunctionInfo {
        std::string file;
        std::string func;
        unsigned int line;
        unsigned int indentCount;
        std::vector<std::string> messages;
        int nWarnings;
        int nErrors;
        bool functionFinished;
        std::chrono::steady_clock::time_point startTime;

        // Updated constructor using member initializer list
        FunctionInfo(const std::string file_in, const std::string func_in, const unsigned int line_in, const unsigned int indentCount_in);
        void addNote(const unsigned int line_in,const std::string message);
        void addWarning(const unsigned int line_in,const std::string message);
        void addError(const unsigned int line_in,const std::string message);
        void addExit(const unsigned int line_in, const std::string message = "");
        bool isOK();
        bool equals(const std::string file_in, const std::string func_in) const;
    };

// data
private:
    static std::vector<FunctionInfo> functions;
    static unsigned int              indentSize;
    static std::string               indentString;
    static unsigned char             currentIndentCount;

    static bool                      on;
    static bool                      ignorePrints;
    static bool                      initialized;
    static bool                      ignoreWarnings;
    static bool                      exitOnError;
    static bool                      printFuncStartAndEnd;
    static bool                      showFilePath;
    static bool                      saveScopes;
    static unsigned char             maxIndentCountToSave;

// functions
public:
    static void init(const bool on = true,
                     const unsigned int indentSize = 4, 
                     const bool ignorePrints = false, 
                     const bool ignoreWarnings = false, 
                     const bool exitOnError = true,
                     const bool printFuncStartAndEnd = false,
                     const bool showFilePath = true,
                     const bool saveScopes = false);
    static void setOn(const bool on);
    static void setIgnorePrint(const bool ignorePrints);
    static void setIndentSize(const unsigned int indentSize);
    static void setIgnoreWarnings(const bool ignoreWarnings);
    static void setExitOnError(const bool exitOnError);
    static void setPrintFuncStartAndEnd(const bool exitOnError);
    static void setShowFilePath(const bool showFilePath);
    static void setSaveScopes(const bool saveScopes, const unsigned char forLayerDepth);
    struct Scope {
        Scope(const unsigned int line_in, const std::string& func_in, const std::string& file_in);
        ~Scope();
    };
    // print(__LINE__);
    static void print(const unsigned int line);
    static void print();
    static void note(const unsigned int line, const std::string message="");
    static void warning(const unsigned int line, const std::string message="");
    static void error(const unsigned int line, const std::string message="");
    static void exit(const unsigned int line, const std::string message="");
    static void printAllInfo();//static void time(const std::string message="");
private:
    static LONG WINAPI exceptionFilter(EXCEPTION_POINTERS* exceptionInfo);
    static void signalHandler(int signum);
    static std::string getIndentString(const int indentCount);
    static std::string getAllInfo();
    static void ensureInitialized();
    static void removeAllreadySavedFunctions(const std::string& file_in, const std::string& func_in);
};

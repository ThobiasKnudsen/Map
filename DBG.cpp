#include "DBG.hpp"

#include <Windows.h>
#include <stdio.h>
#include <cstdlib> // For std::exit
#include <chrono>
#include <csignal>
#include <algorithm>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#include <fmt/core.h>
#pragma GCC diagnostic pop

// Updated constructor using member initializer list
DBG::FunctionInfo::FunctionInfo(const std::string file_in, const std::string func_in, const unsigned int line_in, const unsigned int indentCount_in) : file(file_in), func(func_in), line(line_in), indentCount(indentCount_in), messages({}), nWarnings(0), nErrors(0), functionFinished(false), startTime(std::chrono::steady_clock::now()) {}
void DBG::FunctionInfo::addNote(const unsigned int line_in, const std::string message) 
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    this->messages.push_back(fmt::format("{}{} {}ms NOTE {}", std::string(DBG::indentSize*(this->indentCount+1), ' '), line_in, elapsed, message));
}
void DBG::FunctionInfo::addWarning(const unsigned int line_in, const std::string message) 
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    this->messages.push_back(fmt::format("{}{} {}ms WARNING {}", std::string(DBG::indentSize*(this->indentCount+1), ' '), line_in, elapsed, message));this->nWarnings++;
}
void DBG::FunctionInfo::addError(const unsigned int line_in, const std::string message) 
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    this->messages.push_back(fmt::format("{}{} {}ms ERRROR {}", std::string(DBG::indentSize*(this->indentCount+1), ' '), line_in, elapsed, message));
    this->nErrors++;
}
void DBG::FunctionInfo::addExit(const unsigned int line_in,const std::string message) 
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    this->messages.push_back(fmt::format("{}{} {}ms EXIT {}", std::string(DBG::indentSize*(this->indentCount+1), ' '), line_in, elapsed, message));
    this->messages.push_back(fmt::format("{}{} EXIT {}", std::string(DBG::indentSize*(this->indentCount+1), ' '), line_in, message));
}
//void addTime(const std::string message) {this->messages.push_back("|TIME   |"+message);}
bool DBG::FunctionInfo::isOK() 
{
    return !this->nWarnings && !this->nErrors;
}
bool DBG::FunctionInfo::equals(const std::string file_in, const std::string func_in) const 
{
    return this->file==file_in && this->func==func_in;
}

// private
std::vector<DBG::FunctionInfo> DBG::functions; // Assumes FunctionInfo is a defined struct or class
unsigned int DBG::indentSize;
std::string DBG::indentString;
unsigned char DBG::currentIndentCount;
bool DBG::on;
bool DBG::ignorePrints;
bool DBG::initialized = false;
bool DBG::ignoreWarnings;
bool DBG::exitOnError;
bool DBG::printFuncStartAndEnd;
bool DBG::showFilePath;
bool DBG::saveScopes;
unsigned char DBG::maxIndentCountToSave;


// public
void DBG::init(const bool on,
               const unsigned int indentSize, 
               const bool ignorePrints, 
               const bool ignoreWarnings, 
               const bool exitOnError,
               const bool printFuncStartAndEnd,
               const bool showFilePath,
               const bool saveScopes) 
{
    DBG::initialized = true;
    SetUnhandledExceptionFilter(DBG::exceptionFilter);
    signal(SIGINT, DBG::signalHandler);
    DBG::currentIndentCount = 0;
    DBG::functions = {};
    DBG::setOn(on);
    DBG::setIndentSize(indentSize);
    DBG::setIgnorePrint(ignorePrints);
    DBG::setIgnoreWarnings(ignoreWarnings);
    DBG::setExitOnError(exitOnError);
    DBG::setPrintFuncStartAndEnd(printFuncStartAndEnd);
    DBG::setShowFilePath(showFilePath);
    DBG::setSaveScopes(saveScopes, 0);
}
void DBG::setOn(const bool on) 
{
    ensureInitialized();
    DBG::on = on;
}
void DBG::setIgnorePrint(const bool ignorePrints) 
{
    DBG::ensureInitialized();
    DBG::ignorePrints = ignorePrints;
}
void DBG::setIndentSize(const unsigned int indentSize) 
{
    DBG::ensureInitialized();
    DBG::indentSize = indentSize;
    DBG::indentString = std::string(indentSize, ' '); // More concise and efficient
}
void DBG::setIgnoreWarnings(const bool ignoreWarnings) 
{
    DBG::ensureInitialized();
    DBG::ignoreWarnings = ignoreWarnings;
}
void DBG::setExitOnError(const bool exitOnError) 
{
    DBG::ensureInitialized();
    DBG::exitOnError = exitOnError;
}
void DBG::setPrintFuncStartAndEnd(const bool printFuncStartAndEnd) 
{
    DBG::ensureInitialized();
    DBG::printFuncStartAndEnd = printFuncStartAndEnd;
}// funcStart(__FILE__, __LINE__, __func__);
void DBG::setShowFilePath(const bool showFilePath) 
{
    ensureInitialized();
    DBG::showFilePath = showFilePath;
}
void DBG::setSaveScopes(const bool saveScopes, const unsigned char forLayerDepth)
{
    ensureInitialized();
    DBG::saveScopes = saveScopes;
    DBG::maxIndentCountToSave = DBG::currentIndentCount+forLayerDepth;
}
void DBG::print(const unsigned int line) 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    if (DBG::ignorePrints) {return;}
    DBG::note(line, "printing");
    print();
}
void DBG::print() 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    if (DBG::ignorePrints) {return;}
    std::string printString = getAllInfo();
    printf("%s",printString.c_str());
}
void DBG::note(const unsigned int line, const std::string message) 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    for (int i = static_cast<int>(DBG::functions.size()-1); i >= 0; i--) {
        if (DBG::functions[i].functionFinished) 
        {
            continue;
        }
        DBG::functions[i].addNote(line, message);
        return;
    }

    // function should not end up here
    DBG::Scope scope(__LINE__, __func__, __FILE__);
    DBG::warning(__LINE__, "functionFinished is used wrong");

    DBG::functions[0].addNote(line, message);
}
void DBG::warning(const unsigned int line, const std::string message) 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    for (int i = static_cast<int>(DBG::functions.size()-1); i >= 0; i--) {
        if (DBG::functions[i].functionFinished) 
        {
            continue;
        }
        DBG::functions[i].addWarning(line, message);
        if (DBG::ignoreWarnings) {return;}
        DBG::print();
        break;
    }

    // function should not end up here
    if (DBG::ignoreWarnings) {return;}
    DBG::print();
}
void DBG::error(const unsigned int line, const std::string message) 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    for (int i = static_cast<int>(DBG::functions.size()-1); i >= 0; i--) {
        if (DBG::functions[i].functionFinished) 
        {
            continue;
        }
        DBG::functions[i].addError(line, message);
        if (DBG::exitOnError) {DBG::exit(line);}
        else {DBG::print();}
        return;
    }

    // function should not end up here
    DBG::Scope scope(__LINE__, __func__, __FILE__);
    DBG::warning(__LINE__, "functionFinished is used wrong");
    
    DBG::functions[0].addError(line, message);
    if (DBG::exitOnError) {DBG::exit(line);}
    else {DBG::print();}
}
void DBG::exit(const unsigned int line, const std::string message) 
{
    if (!DBG::on) {return;}
    DBG::ignorePrints = false;
    DBG::initialized = true;
    for (int i = static_cast<int>(DBG::functions.size()-1); i >= 0; i--) {
        if (DBG::functions[i].functionFinished) 
        {
            continue;
        }
        DBG::functions[i].addExit(line, message);
        DBG::print();
        std::exit(EXIT_FAILURE);
        return;
    }

    // function should not end up here
    DBG::Scope scope(__LINE__, __func__, __FILE__);
    DBG::warning(__LINE__, "functionFinished is used wrong");

    DBG::functions[0].addExit(line, message);
    DBG::print();
    std::exit(EXIT_FAILURE);
    //throw std::runtime_error("terminating program");
}
/*
void DBG::time(const std::string message) 
{
    // μs
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - DBG::functions.back().startTime);
    DBG::functions.back().addTime(std::to_string(duration.count())+"microSec"+message);
}
*/
DBG::Scope::Scope(const unsigned int line_in, const std::string& func_in, const std::string& file_in)
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();
    //DBG::removeAllreadySavedFunctions(file_in, func_in);
    DBG::functions.push_back(FunctionInfo(file_in, func_in, line_in, DBG::currentIndentCount));
    DBG::currentIndentCount++;
}
DBG::Scope::~Scope() 
{
    if (!DBG::on) {return;}
    DBG::ensureInitialized();

    DBG::functions.back().messages.push_back(fmt::format("{}FUNCTION DONE", std::string(DBG::currentIndentCount*DBG::indentSize, ' ')));
    DBG::functions.back().functionFinished = true;

    if (DBG::saveScopes && DBG::functions.back().indentCount<=DBG::maxIndentCountToSave) 
    {
        DBG::currentIndentCount--;
        return;
    }
    DBG::functions.pop_back();
    DBG::currentIndentCount--;
}

// private
LONG WINAPI DBG::exceptionFilter(EXCEPTION_POINTERS* exceptionInfo) 
{
    (void)exceptionInfo; // makes unused paramater used
    DBG::ensureInitialized();
    DBG::exit(0, "PROGRAM CRASH");
    //if (exceptionInfo->ExceptionRecord->ExceptionCode == EXCEPTION_ACCESS_VIOLATION){}
    return EXCEPTION_EXECUTE_HANDLER;
}
void DBG::signalHandler(int signum) {
    (void)signum; // makes unused paramater used
 
    DBG::exit(42, "^C terminates the program");
}
std::string DBG::getIndentString(const int indentCount) 
{
    DBG::ensureInitialized();
    std::string indentString = "";
    for (int i = 0; i < static_cast<int>(DBG::indentSize); i++) 
    {
        indentString+=" ";
    }
    std::string returnString = "";
    for (int i = 0; i < indentCount; i++) 
    {
        returnString+=indentString;
    }
    return returnString;
}
std::string DBG::getAllInfo() 
{
    DBG::ensureInitialized();
    std::string allInfo = "\n";
    for (const auto& functionInfo : DBG::functions) {
        allInfo+=fmt::format("{}{} {} {}()\n", std::string(DBG::indentSize*functionInfo.indentCount, ' '), functionInfo.line, functionInfo.file, functionInfo.func);
        for (const auto& message : functionInfo.messages) {
            allInfo+=fmt::format("{}\n", message);
        }
    }
    allInfo += "\n";
    return allInfo;
}
void DBG::ensureInitialized() 
{
    if (!DBG::initialized) 
    {
        DBG::exit(0, "DBG has not been initialized. Use DBG::init()");
    }
}
void DBG::removeAllreadySavedFunctions(const std::string& file_in, const std::string& func_in) 
{
    auto it = std::find_if(DBG::functions.begin(), 
                           DBG::functions.end(), 
                           [&](const DBG::FunctionInfo& functionInfo) {
                               return functionInfo.equals(file_in, func_in);
                           });

    if (it != DBG::functions.end()) {
        auto removeStartIndex = std::distance(DBG::functions.begin(), it);

        if (static_cast<size_t>(removeStartIndex) <= DBG::functions.size()) {
            DBG::functions.erase(DBG::functions.begin() + removeStartIndex, DBG::functions.begin() + DBG::functions.size() + 1);
        }
    }
}
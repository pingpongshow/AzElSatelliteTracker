/**
 * @file utils.h
 * @brief Common utility functions and error handling
 * 
 * Provides shared utilities to avoid code duplication across modules:
 * - URL encoding
 * - Error result types
 * - Common string operations
 * 
 * ERROR HANDLING PATTERN:
 * =======================
 * This codebase uses a standardized error handling pattern based on Result<T>.
 * 
 * For NEW code, use Result<T>:
 *   Result<TLEEntry> fetchTLE(int id) {
 *       if (error) return Result<TLEEntry>::fail("Error message");
 *       return Result<TLEEntry>::ok(entry);
 *   }
 * 
 * For BACKWARD COMPATIBILITY with existing code that uses _lastError:
 *   bool fetchTLE(int id, TLEEntry& entry) {
 *       auto result = fetchTLEInternal(id);
 *       if (!result) {
 *           _lastError = result.error;
 *           return false;
 *       }
 *       entry = result.value;
 *       return true;
 *   }
 * 
 * Or use the RETURN_IF_ERROR and SET_ERROR_RETURN macros for cleaner code.
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// COMMON ERROR RESULT TYPE
// =============================================================================

/**
 * @struct Result
 * @brief Generic result type for operations that can fail
 * 
 * Provides consistent error handling across all modules.
 * Preferred over returning bool and setting _lastError separately.
 * 
 * Usage:
 *   // Creating results
 *   return Result<int>::ok(42);
 *   return Result<int>::fail("Something went wrong");
 * 
 *   // Checking results
 *   auto result = someFunction();
 *   if (result) {
 *       use(result.value);
 *   } else {
 *       handleError(result.error);
 *   }
 */
template<typename T>
struct Result {
    bool success;
    T value;
    String error;
    
    // Factory methods for cleaner creation
    static Result<T> ok(const T& val) {
        Result<T> r;
        r.success = true;
        r.value = val;
        return r;
    }
    
    static Result<T> fail(const String& errorMsg) {
        Result<T> r;
        r.success = false;
        r.error = errorMsg;
        return r;
    }
    
    static Result<T> fail(const char* errorMsg) {
        return fail(String(errorMsg));
    }
    
    // Implicit conversion to bool for easy checking
    operator bool() const { return success; }
    
    // Get value or default if failed
    T valueOr(const T& defaultVal) const {
        return success ? value : defaultVal;
    }
    
    // Transform the value if successful
    template<typename U>
    Result<U> map(U (*transform)(const T&)) const {
        if (success) {
            return Result<U>::ok(transform(value));
        }
        return Result<U>::fail(error);
    }
};

// Specialization for void return type
template<>
struct Result<void> {
    bool success;
    String error;
    
    static Result<void> ok() {
        Result<void> r;
        r.success = true;
        return r;
    }
    
    static Result<void> fail(const String& errorMsg) {
        Result<void> r;
        r.success = false;
        r.error = errorMsg;
        return r;
    }
    
    static Result<void> fail(const char* errorMsg) {
        return fail(String(errorMsg));
    }
    
    operator bool() const { return success; }
};

// Common type aliases
using VoidResult = Result<void>;
using BoolResult = Result<bool>;
using IntResult = Result<int>;
using StringResult = Result<String>;

// =============================================================================
// ERROR HANDLING MACROS
// =============================================================================

/**
 * @brief Return early if a Result indicates failure
 * 
 * Usage in functions returning Result<T>:
 *   Result<Data> result = getData();
 *   RETURN_IF_ERROR(result);  // Returns Result<T>::fail(result.error) if failed
 *   // Continue with result.value...
 */
#define RETURN_IF_ERROR(result) \
    do { \
        if (!(result)) { \
            return decltype(result)::fail((result).error); \
        } \
    } while(0)

/**
 * @brief Set _lastError and return false (for backward-compatible bool methods)
 * 
 * Usage in methods that set _lastError:
 *   if (error_condition) {
 *       SET_ERROR_RETURN("Error message");
 *   }
 * 
 * Expands to:
 *   _lastError = "Error message"; return false;
 */
#define SET_ERROR_RETURN(msg) \
    do { \
        _lastError = (msg); \
        return false; \
    } while(0)

/**
 * @brief Set _lastError and return a specific value
 * 
 * Usage:
 *   if (error) SET_ERROR_RETURN_VAL("Error", -1);
 */
#define SET_ERROR_RETURN_VAL(msg, val) \
    do { \
        _lastError = (msg); \
        return (val); \
    } while(0)

/**
 * @brief Convert a Result to bool, setting _lastError on failure
 * 
 * Usage in backward-compatible wrapper methods:
 *   bool oldMethod(Data& out) {
 *       auto result = newMethodReturningResult();
 *       RESULT_TO_BOOL(result, out);  // Sets _lastError and returns false on failure
 *       return true;
 *   }
 */
#define RESULT_TO_BOOL(result, outVar) \
    do { \
        if (!(result)) { \
            _lastError = (result).error; \
            return false; \
        } \
        (outVar) = (result).value; \
    } while(0)

/**
 * @brief Convert a Result<void> to bool, setting _lastError on failure
 */
#define VOID_RESULT_TO_BOOL(result) \
    do { \
        if (!(result)) { \
            _lastError = (result).error; \
            return false; \
        } \
    } while(0)

// =============================================================================
// URL ENCODING
// =============================================================================

/**
 * @brief URL-encode a string for use in HTTP requests
 * @param str The string to encode
 * @return URL-encoded string
 * 
 * Encodes special characters according to RFC 3986.
 * Safe characters (not encoded): A-Z, a-z, 0-9, -, _, .
 * Space is encoded as %20 (not +)
 */
inline String urlEncode(const char* str) {
    String encoded;
    encoded.reserve(strlen(str) * 2);  // Reserve space to reduce reallocations
    
    for (int i = 0; str[i]; i++) {
        char c = str[i];
        if (isalnum(c) || c == '-' || c == '_' || c == '.') {
            encoded += c;
        } else if (c == ' ') {
            encoded += "%20";
        } else {
            char hex[4];
            snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
            encoded += hex;
        }
    }
    return encoded;
}

/**
 * @brief URL-encode a String for use in HTTP requests
 * @param str The String to encode
 * @return URL-encoded string
 */
inline String urlEncode(const String& str) {
    return urlEncode(str.c_str());
}

// =============================================================================
// STRING UTILITIES
// =============================================================================

/**
 * @brief Safely copy a string with null termination
 * @param dest Destination buffer
 * @param src Source string
 * @param destSize Size of destination buffer
 */
inline void safeCopy(char* dest, const char* src, size_t destSize) {
    if (destSize == 0) return;
    strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0';
}

/**
 * @brief Check if a string is empty or null
 * @param str String to check
 * @return true if null or empty
 */
inline bool isNullOrEmpty(const char* str) {
    return str == nullptr || str[0] == '\0';
}

// =============================================================================
// NUMERIC UTILITIES
// =============================================================================

/**
 * @brief Clamp a value between min and max
 * @param value The value to clamp
 * @param minVal Minimum allowed value
 * @param maxVal Maximum allowed value
 * @return Clamped value
 */
template<typename T>
inline T clamp(T value, T minVal, T maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/**
 * @brief Normalize an angle to the range [0, 360)
 * @param degrees Angle in degrees
 * @return Normalized angle
 */
inline float normalizeAngle(float degrees) {
    float normalized = fmodf(degrees, 360.0f);
    if (normalized < 0) normalized += 360.0f;
    return normalized;
}

/**
 * @brief Calculate the shortest angular distance between two angles
 * @param from Starting angle (degrees)
 * @param to Target angle (degrees)
 * @return Shortest distance (-180 to 180)
 */
inline float shortestAngleDist(float from, float to) {
    float diff = normalizeAngle(to) - normalizeAngle(from);
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

// =============================================================================
// TIME UTILITIES
// =============================================================================

/**
 * @brief Convert milliseconds to human-readable duration string
 * @param ms Milliseconds
 * @return Formatted string like "2h 30m 15s"
 */
inline String formatDuration(uint32_t ms) {
    uint32_t seconds = ms / 1000;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;
    
    seconds %= 60;
    minutes %= 60;
    
    if (hours > 0) {
        return String(hours) + "h " + String(minutes) + "m " + String(seconds) + "s";
    } else if (minutes > 0) {
        return String(minutes) + "m " + String(seconds) + "s";
    } else {
        return String(seconds) + "s";
    }
}

/**
 * @brief Format a Unix timestamp as ISO 8601 string
 * @param unixTime Unix timestamp
 * @return ISO 8601 formatted string
 */
inline String formatUnixTime(uint32_t unixTime) {
    // Simple implementation - for full ISO 8601, use a proper time library
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", unixTime);
    return String(buf);
}

#endif // UTILS_H

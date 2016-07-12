#ifndef UTILITY_STRUTIL_H
#define UTILITY_STRUTIL_H

#include <string>
#include <algorithm>

namespace utility {

namespace strutil {

    // http://stackoverflow.com/a/874160/1387006
    inline bool endsWith(const std::string& str, const std::string& ending) {

        if (str.length() >= ending.length()) {
            return (0 == str.compare (str.length() - ending.length(), ending.length(), ending));
        }
        else {
            return false;
        }
    }

    inline bool startsWith(const std::string& str, const std::string& start) {

        if (str.length() >= start.length()) {
            return (0 == str.compare (0, start.length(), start));
        }
        else {
            return false;
        }
    }

    inline void trimLeft(std::string& str, const std::string& tokens) {
        str.erase(0, str.find_first_not_of(tokens));        // remove tokens from the beginning of the string.
    }

    inline void trimRight(std::string& str, const std::string& tokens) {
        str.erase(str.find_last_not_of(tokens), str.length() - 1);        // remove tokens from the beginning of the string.
    }

    inline void trim(std::string& str, const std::string& tokens) {
        trimLeft(str, tokens);
        trimRight(str, tokens);
    }

    // http://stackoverflow.com/a/237280
    inline std::vector<std::string>& split(const std::string& str, char delimeter, std::vector<std::string>& elements) {
        std::stringstream ss(str);
        std::string item;
        while (std::getline(ss, item, delimeter)) {
            elements.push_back(item);
        }
        return elements;
    }

    inline std::vector<std::string> split(const std::string& string, char delimeter) {
        std::vector<std::string> elements;
        split(string, delimeter, elements);
        return elements;
    }

    inline void removeAll(std::string& str, const std::string& tokens) {
        str.erase(std::remove_if(str.begin(), str.end(),
            [&tokens](const char& c) {
                return tokens.find(c) != std::string::npos;
            }
            ), str.end());        // remove all tokens from the string.
    }
}
}
#endif

#ifndef UTILITY_FILEUTIL_H
#define UTILITY_FILEUTIL_H

#include <string>
#include <array>
#include <vector>
#include <fstream>

namespace utility {
    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    namespace file {
        std::string loadFromFile(const std::string& path);

        template <typename TData>
        void writeToFile(const std::string& path, const TData& data, bool append = false) {
            std::ofstream file(path,
                append
                    ? std::ios::out | std::ios::app
                    : std::ios::out | std::ios::trunc);
            file << data;
        }

        bool createDir(const std::string& path);

        bool exists(const std::string& path);

        bool isDir(const std::string& path);

        std::vector<std::string> listDir(const std::string& path);

        /**
         * @brief Splits a path into it's basename and dirname components.
         *
         * @param input the input string
         *
         * @return the dirname and basename in the posix style
         */
        std::pair<std::string, std::string> pathSplit(const std::string& input);

        /**
         * Finds and returns a list of file paths given a specified directory. This function is able to include any
         * sub-directories and their file paths if recursive is set to true.
         *
         * @param directory The directory to base the search off.
         * @param recursive Whether the directories within the specified directory is searched or not.
         * @return The list of file paths within a specified directory.
         */
        std::vector<std::string> listFiles(const std::string& directory, bool recursive = false);
    }
}
#endif

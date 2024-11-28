#ifndef DATALOADER_HPP
#define DATALOADER_HPP

#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>

namespace fs = std::filesystem;

/**
 * @brief Comparator function to sort files based on the timestamp in the filename
 * 
 * @param a File path a
 * @param b File path b
 * @return true if a < b
 * @return false if a >= b
 */
bool filename_timestamp_comparator( const fs::path &a, const fs::path &b) {
    return std::stod(a.stem().string()) < std::stod(b.stem().string());
}

/**
 * @brief Dataloader class to load files from a directory or a single file
 */
class Dataloader {
    public:
        Dataloader();
        ~Dataloader();

        void loadFile(const std::string &input_path);
        void loadDir(const std::string &input_path, const std::string &extension = ".pcd");
        size_t size();

        std::vector<fs::path> getFiles();

        std::vector<fs::path> getSampledFiles(const size_t& n_samples = 10);

        std::vector<fs::path> getTimeSampledFiles(const float& delta_time_th = 1.0);

    private:
        std::vector<fs::path> files;
};

Dataloader::Dataloader() {
    this->files.clear();
}

Dataloader::~Dataloader() {
    this->files.clear();
}

/**
 * @brief Returns the number of files loaded
 */
size_t
Dataloader::size() {
    return this->files.size();
}

/**
 * @brief Load all files with the given extension from the input directory
 * 
 * @param input_path Input directory path
 * @param extension File extension to load
 */
void
Dataloader::loadDir(const std::string &input_path, const std::string &extension) {

    for (const auto &entry : fs::directory_iterator(input_path)) {
        if (entry.path().extension() == extension) {
            this->files.push_back(entry.path());
        }
    }
}

/**
 * @brief Load a single file
 * 
 * @param input_path Input file path
 */
void
Dataloader::loadFile(const std::string &input_path) {
    this->files.push_back(input_path);
}

/**
 * @brief Returns the loaded files
 * @return std::vector<fs::path> Loaded files
 */
std::vector<fs::path> 
Dataloader::getFiles() {
    return this->files;
}

/**
 * @brief Returns n_samples random sampled files
 * 
 * @param n_samples Number of random samples to return
 * @return std::vector<fs::path> Random sampled files
 */
std::vector<fs::path>
Dataloader::getSampledFiles(const size_t& n_samples) {
    std::vector<fs::path> sampled_files;

    if (n_samples > this->files.size()) {
        return this->files;
    }

    for (size_t i = 0; i < n_samples; i++) {
        int idx = rand() % this->files.size();
        sampled_files.push_back(this->files[idx]);
    }

    return sampled_files;
}


/**
 * @brief Returns files sampled based on the time difference between consecutive files
 * 
 * @param delta_time_th Time difference threshold
 * @return std::vector<fs::path> Time sampled files
 */
std::vector<fs::path>
Dataloader::getTimeSampledFiles(const float& delta_time_th) {
    std::vector<fs::path> sampled_files;

    if (delta_time_th < 0) {
        return this->files;
    }

    std::sort(this->files.begin(), this->files.end(), filename_timestamp_comparator);
    
    double prev_time = 0.0;
    for (const auto &file : this->files) {

        if (prev_time == 0.0) {
            sampled_files.push_back(file);
            prev_time = std::stod(file.stem().string());
            continue;
        }

        double curr_time = std::stod(file.stem().string());

        if ((curr_time-prev_time) >= delta_time_th) {
            sampled_files.push_back(file);
            prev_time = curr_time;
        }
    }

    return sampled_files;
}

#endif // DATALOADER_HPP
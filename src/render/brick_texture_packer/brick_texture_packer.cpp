/*
Copyright (c) 2016 Ryan L. Guy

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h>
#include <iostream>
#include <getopt.h>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "lodepng/lodepng.h"

int START_NUMBER = 0;
std::string INPUT_FORMAT = "%04d.data";
std::string OUTPUT_FORMAT = "%04d.png";

bool set_start_number(std::string s) {
    int n;
    std::istringstream iss(s);

    iss >> n;
    if(iss.fail() || n < 0) {
        std::cout << "Error: 'start_number' must be a positive integer: " << s << std::endl;
        return false;
    }

    START_NUMBER = n;

    return true;
}

bool set_input_format(std::string s) {
    INPUT_FORMAT = s;
    return true;
}

bool set_output_format(std::string s) {
    OUTPUT_FORMAT = s;
    return true;
}

bool parse_options(int argc, char **argv) {
    while (true) {
        int option_index = 0;
        int c;
        static struct option long_options[] = {
            {"start_number",   required_argument, 0, 's' },
            {"input",          required_argument, 0, 'i' },
            {"output",         required_argument, 0, 'o' },
            {0,                0,                 0,  0  }
        };

        c = getopt_long(argc, argv, "s:i:o:", long_options, &option_index);
        if (c == -1)
            break;

        switch (c) {

            case 's':
                if (!set_start_number(optarg)) { return false; }
                break;

            case 'i':
                if (!set_input_format(optarg)) { return false; }
                break;

            case 'o':
                if (!set_output_format(optarg)) { return false; }
                break;

            default:
                return false;
        }
    }

    return true;
}

std::string get_formatted_filename(std::string format, int number) {
    char buffer[1024];
    snprintf(buffer, sizeof(buffer), format.c_str(), number);

    return std::string(buffer);
}

int get_filesize(std::ifstream *file){

    std::streampos current_position = file->tellg();

    std::streampos fsize = 0;
    file->seekg(0, std::ios::beg);
    fsize = file->tellg();
    file->seekg(0, std::ios::end);
    fsize = file->tellg() - fsize;

    file->seekg(current_position, std::ios::beg);

    return (int)fsize;
}

void get_texture_dimensions(int num_pixels, int *width, int *height) {
    *width = ceil(sqrt(num_pixels));
    *height = ceil((float)num_pixels / (float)*width);
}

std::vector<unsigned char> pack_texture_data(std::ifstream *datafile){

    std::vector<unsigned char> data;
    int filesize = get_filesize(datafile);
    data.resize(filesize);

    datafile->seekg(0, std::ios_base::beg);
    datafile->read((char*)&data[0], filesize);

    int imgwidth, imgheight;
    get_texture_dimensions(data.size(), &imgwidth, &imgheight);

    std::vector<std::vector<unsigned char> > imgdata;
    imgdata.reserve(imgheight);

    for (int i = 0; i < imgheight; i++) {
        std::vector<unsigned char> row(4*imgwidth, (unsigned char)0);
        imgdata.push_back(row);
    }

    int offset = 0;
    for (unsigned int j = 0; j < imgdata.size(); j++) {
        for (unsigned int i = 0; i < imgdata[0].size(); i += 4) {

            unsigned char r = 255;
            unsigned char g = 0;
            unsigned char b = 0;
            unsigned char alpha = 255;
            if (offset < (int)data.size()) {
                r = data[offset];
                g = data[offset];
                b = data[offset];
            }

            imgdata[j][i]   = r;
            imgdata[j][i+1] = g;
            imgdata[j][i+2] = b;
            imgdata[j][i+3] = alpha;

            offset++;
        }
    }
    std::reverse(imgdata.begin(), imgdata.end());

    std::vector<unsigned char> image;
    image.reserve(4*imgwidth*imgheight);

    for (unsigned int j = 0; j < imgdata.size(); j++) {
        for (unsigned int i = 0; i < imgdata[0].size(); i++) {
            image.push_back(imgdata[j][i]);
        }
    }

    return image;
}

bool write_texture(std::string filename, std::vector<unsigned char> &image) {
    int imgwidth, imgheight;
    get_texture_dimensions(image.size() / 4, &imgwidth, &imgheight);

    unsigned int error = lodepng::encode(filename, image, imgwidth, imgheight);

    if(error) {
        std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
        return false;
    }

    return true;
}

void print_usage() {
    std::cout << std::endl;
    std::cout << "USAGE:" << std::endl;
    std::cout << "./brick_texture_packer --start_number [first_frame_number] --input [input_format_string] --output [output_format_string]" << std::endl << std::endl;
    std::cout << "EXAMPLE USAGE:" << std::endl;
    std::cout << "./brick_texture_packer --start_number 30 --input bricktexture%06d.data --output texture%04d.png" << std::endl;
}

int main(int argc, char **argv) {

    if (!parse_options(argc, argv)) {
        std::cout << "FAILED" << std::endl;
        return 0;
    }

    int current_frame = START_NUMBER;
    int num_files_processed = 0;
    int datafilesize = 0;
    std::string last_processed_filename;
    while (true) {

        std::string input_filename = get_formatted_filename(INPUT_FORMAT, current_frame);
        std::string output_filename = get_formatted_filename(OUTPUT_FORMAT, current_frame);

        if ( input_filename == last_processed_filename) {
            std::cout << "ERROR: already proccessed file: " << input_filename << std::endl;
            print_usage();
            return 0;
        }

        std::ifstream datafile;
        datafile.open(input_filename.c_str(), std::ios::in | std::ios::binary);

        if (datafile.fail()) {
            if (num_files_processed == 0) {
                std::cout << "ERROR: unable to open input file: " << input_filename << std::endl;
                print_usage();
            }
            return 0;
        }

        if (datafile.eof()) {
            std::cout << "ERROR: input file is empty: " << input_filename << std::endl;
            print_usage();
            return 0;
        }

        std::cout << "Proccessing file:\t" << input_filename  << std::endl;

        int size = get_filesize(&datafile);
        if (num_files_processed == 0) {
            datafilesize = get_filesize(&datafile);
        } else if (size != datafilesize) {
            std::cout << "ERROR: all filesizes must be the same." << std::endl;
            std::cout << "Current filesize:  " << size << std::endl;
            std::cout << "Expected filesize: " << datafilesize << std::endl;

            print_usage();
            return 0;
        }
        
        std::vector<unsigned char> image = pack_texture_data(&datafile);
        if (!write_texture(output_filename, image)) {
            std::cout << "ERROR: data was not successfully encoded." << std::endl;

            print_usage();
            return 0;
        }

        std::cout << "Wrote texture file:\t" << output_filename  << std::endl << std::endl;

        current_frame++;
        num_files_processed++;
        last_processed_filename = input_filename;

        datafile.close();
    }

   return 0;
}
#ifndef LOG_WRITER_H
#define LOG_WRITER_H

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class LogWriter {
public:
    // Constructors
    LogWriter(bool html=false);

    // Destructor
    virtual ~LogWriter();

    // LogWriter functions
    void open(const std::string& file_name);
    void close();
    void write(const std::string& s);
    void write(std::stringstream& ss);

    // Helper functions.
    void write_html_document_header();
    void write_html_document_footer();

    void write_html_details_header(const std::string& s);
    void write_html_details_header(std::stringstream& ss);
    void write_html_details_footer();

    // LogWriter properties
    std::ofstream   log_stream;
    bool            html;
};

#endif LOG_WRITER_H

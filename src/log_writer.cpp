
#include "log_writer.h"
#include <string>
#include <iostream>
#include <sstream>

// The global LogWriter object.
LogWriter   log_file;

// Constructors
LogWriter::LogWriter(bool html) : html(html) {}

// Destructor
LogWriter::~LogWriter() {
    close();
}

// Functions
void LogWriter::open(const std::string& file_name) {
    if (file_name.length() > 0) {
        log_stream.open(file_name);
        if (html)
            write_html_document_header();
    }
}

void LogWriter::close() {
    if (log_stream.is_open()) {
        if (html)
            write_html_document_footer();
        log_stream.close();
    }
}

void LogWriter::write(const std::string& s) {
    if (html)
        log_stream << "    <p>" << s << "</p>" << std::endl;
    else
        log_stream << s << std::endl;
}

void LogWriter::write(std::stringstream& ss) {
    write(ss.str());
    ss.str(std::string());
}

// Helper Functions
void LogWriter::write_html_document_header() {
    if (!log_stream.is_open())
        return;
    log_stream
        << "<!DOCTYPE html>" << std::endl
        << "<html lang='en'>" << std::endl
        << "  <head>" << std::endl
        << "    <meta charset='utf-8'>" << std::endl
        << "  </head>" << std::endl
        << "  <body>" << std::endl;
}
void LogWriter::write_html_document_footer() {
    if (!log_stream.is_open())
        return;
    log_stream
        << "  </body>" << std::endl
        << "</html>" << std::endl;
}

void LogWriter::write_html_details_header(const std::string& summary) {
    if (!log_stream.is_open())
        return;
    if (html)
        log_stream 
            << "    <details>" << std::endl
            << "      <summary>" << summary << "</summary>" << std::endl;
    else
        log_stream
            << summary << std::endl;
}
void LogWriter::write_html_details_header(std::stringstream& ss) {
    write_html_details_header(ss.str());
    ss.str(std::string());
}
void LogWriter::write_html_details_footer() {
    if (!log_stream.is_open())
        return;
    if (html)
        log_stream
            << "    </details>" << std::endl;
}

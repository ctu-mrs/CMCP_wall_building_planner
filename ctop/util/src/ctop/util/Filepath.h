//
// Created by Michal Němec on 14/04/2018.
//

#ifndef CTOP_PLANNER_FILEPATH_H
#define CTOP_PLANNER_FILEPATH_H


#include <string>
#include <ostream>

namespace ctop {

class Filepath {

    template<class T>
    T base_name(T const & path, T const & delims = "/\\")
    {
        return path.substr(path.find_last_of(delims) + 1);
    }
    template<class T>
    T remove_extension(T const & filename)
    {
        typename T::size_type const p(filename.find_last_of('.'));
        return p > 0 && p != T::npos ? filename.substr(0, p) : filename;
    }

    template<class T>
    T base_path(T const & filename)
    {
        typename T::size_type const p(filename.find_last_of("/\\"));
        return p > 0 && p != T::npos ? filename.substr(0, p) : filename;
    }

    template<class T>
    T base_extenstion(T const & fn)
    {

        typename T::size_type const p(fn.find_last_of("."));
        return p > 0 && p != T::npos ? fn.substr(p+1) : "";
    }

    void refresh() {
        full_name_ = name_ + "." +ext_;
        full_path_ = path_ + "/" + full_name_;
    }

    std::string path_;
    std::string full_name_;
    std::string name_;
    std::string ext_;
    std::string full_path_;
public:

    Filepath(const std::string& fp) :
            path_(base_path(fp)),
            full_name_(base_name(fp)),
            name_(remove_extension(full_name_)),
            ext_(base_extenstion(full_name_)),
            full_path_(fp)
    {}

    void print(std::ostream& oss) const {
        oss << "full path: " << full_path_ << "\n"
            << "path : " << path_ << "\n"
            << "full name : " << full_name_ << "\n"
            << "name : " << name_ << "\n"
            << "ext : " << ext_ << "\n" ;
    }

    const std::string& path() {
        return path_;
    }

    const std::string& full_name() {
        return full_name_;
    }

    const std::string& name() {
        return name_;
    }

    const std::string& ext() {
        return ext_;
    }

    const std::string& full_path() {
        return full_path_;
    }

    void set_name(const std::string& name) {
        name_ = name;
        refresh();
    }

    void set_ext(const std::string& ext) {
        ext_ = ext;
        refresh();
    }

    void set_path(const std::string& path) {
        path_ = path;
        refresh();
    }
};

}

#endif //UAV_LOCALIZATION_CORE_FILEPATH_H

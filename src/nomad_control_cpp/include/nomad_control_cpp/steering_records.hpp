#pragma once
#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <mutex>
#include <cmath>
#include <cctype>
#include <algorithm>
#include <random>
#include <sstream>
#include <iomanip>

namespace nomad {
// 실측 원본 기록 저장소. 주행 설정을 수정하지 않으며 기존 측정값을 덮어쓰지 않습니다.
class SteeringRecords {
public:
    explicit SteeringRecords(std::filesystem::path directory) : directory_(std::move(directory)) {}
    static std::string newId() {
        std::random_device random;
        std::ostringstream id;
        for(int i=0;i<4;++i)id<<std::hex<<std::setw(8)<<std::setfill('0')<<random();
        return id.str();
    }
    nlohmann::json list() {
        std::lock_guard<std::mutex> lock(mutex_);
        nlohmann::json rows=nlohmann::json::array();int invalid=0;
        if (std::filesystem::exists(directory_)) {
            std::vector<std::filesystem::path> paths;
            for (const auto& entry:std::filesystem::directory_iterator(directory_))
                if(entry.is_regular_file()&&entry.path().extension()==".json") paths.push_back(entry.path());
            std::sort(paths.begin(),paths.end());
            for(const auto& path:paths) {
                if(rows.size()>=1000) break;
                try {std::ifstream in(path);auto row=nlohmann::json::parse(in);validate(row);rows.push_back(row);}
                catch(const std::exception&){++invalid;}
            }
        }
        return {{"accepted",true},{"records",rows},{"invalid_files",invalid},{"applied",false}};
    }
    nlohmann::json save(nlohmann::json row) {
        validate(row);
        std::lock_guard<std::mutex> lock(mutex_);
        std::filesystem::create_directories(directory_);
        const auto path=directory_/(row.at("id").get<std::string>()+".json");
        if(std::filesystem::exists(path)) {
            std::ifstream in(path);auto old=nlohmann::json::parse(in);
            // 동일 요청 재전송은 중복 파일을 만들지 않습니다. 측정 내용을 바꾸어 덮어쓰지는 못합니다.
            auto compare=old;compare.erase("saved_at_s");auto candidate=row;candidate.erase("saved_at_s");
            if(compare!=candidate) throw std::runtime_error("measurement ID already used; reload records before saving again");
            return {{"accepted",true},{"record",old},{"applied",false}};
        }
        size_t count=0;
        for(const auto& entry:std::filesystem::directory_iterator(directory_))
            if(entry.path().extension()==".json")++count;
        if(count>=1000)throw std::runtime_error("measurement storage limit reached (1000 records)");
        const auto temporary=directory_/(row.at("id").get<std::string>()+".tmp");
        {std::ofstream out(temporary,std::ios::binary|std::ios::trunc);out<<row.dump(2)<<'\n';out.flush();
         if(!out)throw std::runtime_error("measurement write failed");}
        std::filesystem::rename(temporary,path);
        return {{"accepted",true},{"record",row},{"applied",false}};
    }
private:
    static void validate(const nlohmann::json& row) {
        const auto id=row.at("id").get<std::string>();
        if(id.size()!=32||!std::all_of(id.begin(),id.end(),[](unsigned char c){return std::isxdigit(c);}))
            throw std::runtime_error("invalid measurement ID");
        for(const auto* key:{"servo_command","saved_at_s"})
            if(!std::isfinite(row.at(key).get<double>()))throw std::runtime_error("non-finite measurement");
        if(row.at("servo_command").get<double>()<0||row.at("servo_command").get<double>()>1||
           row.at("saved_at_s").get<double>()<=0||row.value("applied",true)||
           row.at("note").get<std::string>().size()>600)
            throw std::runtime_error("invalid steering measurement range or confirmation");
        const auto kind=row.value("kind",std::string("measured"));
        if(kind=="command"||kind=="observation") {
            // 명령 기록을 각도 측정값으로 오인하지 않도록 숫자 0 대신 null을 강제합니다.
            for(const auto* key:{"left_deg","right_deg","left_rad","right_rad"})
                if(!row.at(key).is_null())throw std::runtime_error("unmeasured angles must be null");
            if(row.value("measured_confirmed",true))throw std::runtime_error("command is not a measurement");
            if(kind=="command") {
                const auto observed=row.at("observed_servo_command").get<double>();
                if(!std::isfinite(observed)||observed<0||observed>1)
                    throw std::runtime_error("invalid command snapshot");
            }
        } else if(kind=="measured") {
            if(!row.value("measured_confirmed",false))throw std::runtime_error("measurement confirmation required");
            for(const auto* key:{"left_deg","right_deg"})
                if(!std::isfinite(row.at(key).get<double>())||std::abs(row.at(key).get<double>())>85)
                    throw std::runtime_error("invalid measured angle");
        } else throw std::runtime_error("unknown steering record kind");
    }
    std::filesystem::path directory_;
    std::mutex mutex_;
};
} // namespace nomad

#include "nomad_control_cpp/steering_records.hpp"
#include <chrono>
#include <iostream>
#include <limits>

int main(){
    int checks=0,failures=0;
    auto check=[&](bool ok,const char* text){++checks;if(!ok){++failures;std::cerr<<text<<'\n';}};
    auto folder=std::filesystem::temp_directory_path()/("nomad_records_test_"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    nomad::SteeringRecords records(folder);
    nlohmann::json row={{"id",std::string(32,'a')},{"servo_command",.505},{"left_deg",4.2},{"right_deg",3.8},
        {"saved_at_s",1000.0},{"note","test only"},{"measured_confirmed",true},{"applied",false}};
    check(records.list()["records"].empty(),"initial store empty");
    check(records.save(row)["accepted"],"measurement saved");
    auto retry=row;retry["saved_at_s"]=1001;
    check(records.save(retry)["accepted"]&&records.list()["records"].size()==1,"retry idempotent");
    nomad::SteeringRecords reopened(folder);
    check(reopened.list()["records"][0]["left_deg"]==4.2,"record survives store restart");
    for(int i=0;i<7;++i){
        auto bad=row;bad["id"]=std::string(32,'b');
        if(i==0)bad["left_deg"]=90;
        if(i==1)bad["servo_command"]=1.01;
        if(i==2)bad["id"]="../outside";
        if(i==3)bad["measured_confirmed"]=false;
        if(i==4)bad["applied"]=true;
        if(i==5)bad["right_deg"]=std::numeric_limits<double>::quiet_NaN();
        if(i==6){bad=row;bad["left_deg"]=5;}
        bool rejected=false;try{records.save(bad);}catch(const std::exception&){rejected=true;}
        check(rejected,"invalid range/path/unconfirmed/overwrite rejected");
    }
    check(records.list()["records"].size()==1,"invalid attempts preserve original");
    {std::ofstream broken(folder/"broken.json");broken<<"bad json";}
    check(records.list()["invalid_files"]==1&&records.list()["records"].size()==1,"corrupt record reported without hiding valid data");
    auto command=row;command["id"]=nomad::SteeringRecords::newId();command["kind"]="command";
    command["measured_confirmed"]=false;command["observed_servo_command"]=.5;
    for(const auto* key:{"left_deg","right_deg","left_rad","right_rad"})command[key]=nullptr;
    check(records.save(command)["record"]["left_deg"].is_null(),"automatic command stores unknown angle as null");
    check(records.save(command)["accepted"]&&records.list()["records"].size()==2,"command retry does not duplicate");
    auto note=command;note["id"]=nomad::SteeringRecords::newId();note["kind"]="observation";
    check(records.save(note)["accepted"],"observation saved without measured angles");
    for(int i=0;i<3;++i){auto bad=command;bad["id"]=nomad::SteeringRecords::newId();
        if(i==0)bad["left_deg"]=0;if(i==1)bad["measured_confirmed"]=true;if(i==2)bad["kind"]="invented";
        bool rejected=false;try{records.save(bad);}catch(const std::exception&){rejected=true;}
        check(rejected,"command cannot masquerade as actual angle");
    }
    if(folder.parent_path()==std::filesystem::temp_directory_path())std::filesystem::remove_all(folder);
    std::cout<<checks<<" steering record checks, "<<failures<<" failures\n";return failures?1:0;
}

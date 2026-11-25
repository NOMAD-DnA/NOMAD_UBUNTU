#include "nomad_planning_cpp/status_diagnostics.hpp"
#include "nomad_planning_cpp/lpp_diagnostics.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>

namespace npc = nomad_planning_cpp;
static int checks = 0;
void check(bool ok, const char* name) {
  ++checks;
  if (!ok) throw std::runtime_error(name);
}
npc::StatusSample healthy() {
  npc::StatusSample s;
  s.present=true; s.state=1; s.stamp_ns=100000000000LL;
  s.received_wall=50; s.evaluated_wall=50.1; s.evaluated_ros=100.1;
  s.valid_for=.3; s.timeout=1.5; s.future_tolerance=.2;
  return s;
}
bool original_status_ready(const npc::StatusSample& s) {
  if (!s.present || s.received_wall<=0 || s.evaluated_wall-s.received_wall>s.timeout) return false;
  const double source=s.stamp_ns*1e-9;
  return source>0 && source<=s.evaluated_ros+s.future_tolerance
      && s.evaluated_ros-source<=s.timeout && s.state==1
      && s.valid_for>0 && s.evaluated_ros-source<=s.valid_for;
}
int main() {
  const auto inspect=[](npc::StatusSample s){return npc::diagnose_status(std::move(s),1);};
  auto s=healthy(); check(inspect(s).ready(),"healthy");
  s.present=false; check(std::string(inspect(s).category())=="MISSING","missing category");
  check(inspect(s).failures==npc::StatusDiagnostic::no_message,"missing has no invented timestamp failures");
  s=healthy();s.state=0;check(std::string(inspect(s).cause())=="STATE_NOT_READY","WAITING");
  s.state=2;check(std::string(inspect(s).category())=="STATE_NOT_READY","STOPPED");
  s.state=3;check(std::string(inspect(s).category())=="STATE_NOT_READY","FAULT");
  s=healthy();s.received_wall=48;check(std::string(inspect(s).cause())=="RECEIVE_TIMEOUT","receive timeout");
  s=healthy();s.stamp_ns=98000000000LL;check(std::string(inspect(s).cause())=="SOURCE_TIMEOUT","source timeout");
  s=healthy();s.valid_for=.05;check(std::string(inspect(s).cause())=="LEASE_EXPIRED","short lease");
  s.valid_for=0;check(std::string(inspect(s).cause())=="VALID_FOR_INVALID","zero lease");
  s=healthy();s.stamp_ns=0;check(std::string(inspect(s).cause())=="STAMP_INVALID","zero stamp");
  s=healthy();s.stamp_ns=101000000000LL;check(std::string(inspect(s).cause())=="STAMP_IN_FUTURE","future stamp");
  s=healthy();s.state=0;s.received_wall=48;
  const auto overlap=inspect(s);
  check(std::string(overlap.category())=="TIMING_INVALID","gate order for overlapping failures");
  check(overlap.failure_names()=="RECEIVE_TIMEOUT|STATE_NOT_READY","retain secondary state failure");
  s=healthy();s.state=0;s.valid_for=.05;
  check(std::string(inspect(s).category())=="STATE_NOT_READY","state precedes lease in existing gate");

  // Verify boundary/overlap classification agrees with the unchanged gate,
  // including exactly-on-boundary acceptance and receive-vs-source clocks.
  for(bool present:{false,true})for(int state:{0,1,2,3,255})
   for(double received:{-1.,0.,48.,48.5,48.500001,50.,51.})
    for(std::int64_t stamp:{0LL,-1LL,98000000000LL,98500000000LL,98500000001LL,
                           99900000000LL,100000000000LL,100200000000LL,100200000001LL,101000000000LL})
     for(double valid_for:{-1.,0.,.1,.3,1.5,2.}) {
      s=healthy();s.present=present;s.state=state;s.received_wall=received;
      s.stamp_ns=stamp;s.valid_for=valid_for;s.evaluated_ros=100.;s.evaluated_wall=50.;
      check(inspect(s).ready()==original_status_ready(s),"diagnostic must match existing readiness gate");
     }

  s=healthy();s.valid_for=.05;const auto failed=inspect(s);
  const auto current=inspect(healthy());
  auto evidence=npc::status_failure_evidence(current,failed);
  check(evidence.from_worker,"retain worker origin after input recovery");
  check(evidence.failure.sample.valid_for==.05,"retain original worker evidence");
  s=healthy();s.state=0;const auto live=inspect(s);
  evidence=npc::status_failure_evidence(live,failed);
  check(!evidence.from_worker&&evidence.failure.sample.state==0,"latest failed input takes precedence");

  npc::LppDiagnostics logger;
  const auto update=[&](const npc::StatusFailureEvidence& e){
    return logger.update(true,false,false,1,0,"STALE","PERCEPTION_NOT_READY | perception="+e.key());
  };
  check(update({failed,false})==npc::LppDiagnostics::Event::pause,"initial pause");
  for(int j=0;j<200;++j){
    auto later=failed;later.sample.evaluated_ros+=j*.001;later.sample.detail="changing details";
    check(update({later,false})==npc::LppDiagnostics::Event::none,"no repeated log for changing ages or text");
  }
  check(update({live,false})==npc::LppDiagnostics::Event::pause,"subcause transition is logged");
  check(update({live,true})==npc::LppDiagnostics::Event::pause,"worker provenance transition is logged");
  check(logger.update(true,false,true,1,0,"READY","tracking")==npc::LppDiagnostics::Event::recovered,"recovery");
  check(update({failed,false})==npc::LppDiagnostics::Event::pause,"new failure after recovery");
  std::cout<<"status diagnostics: "<<checks<<" checks passed\n";
}

#define NOMAD_DESKTOP_TEST
#include "main.cpp"
#include <QtTest>
#include <iostream>

class FakeController : public QTcpServer {
public:
    QList<QJsonObject> commands;
    QString state="IDLE";
    double erpm=0,servo=.5;
    bool failStatus=false,planning=false,setupAcknowledged=true;
    bool failUpdates=false,failStops=false;
    int startDelay=0,statusDelay=0;
    QString token="fake-only";
    QJsonValue maximum=1850;
    FakeController(){
        connect(this,&QTcpServer::newConnection,this,[this]{
            while(hasPendingConnections()){
                auto* socket=nextPendingConnection();auto bytes=std::make_shared<QByteArray>();
                connect(socket,&QTcpSocket::disconnected,socket,&QObject::deleteLater);
                connect(socket,&QTcpSocket::readyRead,socket,[this,socket,bytes]{
                    bytes->append(socket->readAll());const auto split=bytes->indexOf("\r\n\r\n");if(split<0)return;
                    const QByteArray headers=bytes->left(split);int length=0;
                    for(const auto& line:headers.split('\n'))if(line.toLower().startsWith("content-length:"))length=line.mid(15).trimmed().toInt();
                    if(bytes->size()<split+4+length)return;
                    QJsonObject result;int code=200,delay=0;
                    if(headers.startsWith("GET /api/status")){
                        code=failStatus?503:200;
                        delay=statusDelay;
                        result={{"token",token},{"server_time_s",QDateTime::currentMSecsSinceEpoch()/1000.0},
                            {"control_units","raw"},{"state",state},{"connected",true},{"telemetry_fresh",true},
                            {"stationary",erpm==0},{"driver_ready",true},{"hardware_timeout_verified",true},
                            {"commissioning_setup_acknowledged",setupAcknowledged},{"continuous_manual_supported",true},
                            {"servo_min",0},{"servo_max",1},{"servo_center",.5},{"commanded_servo_position",servo},
                            {"commanded_motor_erpm",erpm},{"raw_erpm",erpm},{"current_a",0},{"voltage_v",16.0},
                            {"motor_max_erpm",maximum},{"wifi_manual_profile",true},{"commissioning",true},
                            {"manual_timeout_s",2},{"command_max_age_s",1.5},{"browser_freshness_s",1.5},
                            {"browser_request_timeout_ms",1500},{"motor_control_mode","vesc_speed_pid"},
                            {"speed_pid_min_erpm_verified",true},{"speed_pid_min_erpm",50},
                            {"planning_supported",planning},{"planning_ready",planning},{"planning_calibration_verified",planning}};
                    }else{
                        auto command=QJsonDocument::fromJson(bytes->mid(split+4,length)).object();
                        command["route"]=QString::fromUtf8(headers.split(' ').at(1));commands.append(command);
                        const int action=command["action"].toInt();
                        const bool accepted=!(failUpdates&&action==2)&&!(failStops&&(action==3||command["route"]=="/api/stop"));
                        if(accepted&&(action==1||action==2)){state="RUNNING";erpm=command["motor_erpm"].toDouble();servo=command["servo_position"].toDouble();}
                        if(accepted&&(action==3||command["route"]=="/api/stop")){state="STOPPED_OUTPUT";erpm=0;}
                        if(command["route"]=="/api/reset")state="IDLE";
                        result={{"accepted",accepted}};if(action==1)delay=startDelay;
                    }
                    auto body=QJsonDocument(result).toJson(QJsonDocument::Compact);
                    const auto response="HTTP/1.1 "+QByteArray::number(code)+" OK\r\nContent-Type: application/json\r\nConnection: close\r\nContent-Length: "+QByteArray::number(body.size())+"\r\n\r\n"+body;
                    QTimer::singleShot(delay,socket,[socket,response]{socket->write(response);socket->disconnectFromHost();});bytes->clear();
                });
            }
        });
        if(!listen(QHostAddress::LocalHost,0))throw std::runtime_error("test HTTP bind failed");
    }
};
class ConsoleTest {
public:
    static int run(){
        int failures=0,checks=0;
        auto expect=[&](bool ok,const char* label){++checks;if(!ok){++failures;std::cerr<<"FAIL "<<label<<'\n';}};
        FakeController server;Console window(false,server.serverPort());
        QTest::qWait(350);
        expect(window.ready()&&window.startButton->isEnabled(),"ready with real HTTP parsing");
        expect(server.commands.empty(),"opening window never starts");
        server.setupAcknowledged=false;
        (void)QTest::qWaitFor([&]{return !window.flag("commissioning_setup_acknowledged");},1500);
        expect(!window.startButton->isEnabled()&&window.connection->text().contains("관찰 전용"),"view-only mode explains locked controls");
        window.begin();QTest::qWait(100);
        expect(server.commands.empty(),"view-only START sends no command");
        server.setupAcknowledged=true;
        (void)QTest::qWaitFor([&]{return window.ready();},1500);
        const auto args=qApp->arguments();const int snapshot=args.indexOf("--snapshot");
        if(snapshot>=0&&snapshot+1<args.size()){
            window.show();QTest::qWait(100);window.grab().save(args[snapshot+1]);window.hide();
        }
        window.begin();QTest::qWait(250);
        expect(window.active&&server.erpm==0,"explicit START zero motor");
        window.speed->setValue(550);window.steering->setValue(1000);QTest::qWait(200);
        expect(server.erpm==550&&server.servo==0,"right GUI endpoint maps to raw servo zero");
        window.inputMode->setCurrentIndex(1);QTest::qWait(400);
        expect(!window.active&&!window.stopping&&server.erpm==0,"mode switch STOP and acknowledgement fence");
        window.begin();(void)QTest::qWaitFor([&]{return window.active;},1500);
        QKeyEvent up(QEvent::KeyPress,Qt::Key_W,Qt::NoModifier);window.eventFilter(&window,&up);(void)QTest::qWaitFor([&]{return server.erpm==50;},1000);
        expect(server.erpm==50,"keyboard speed step");
        QKeyEvent left(QEvent::KeyPress,Qt::Key_A,Qt::NoModifier);window.eventFilter(&window,&left);QTest::qWait(150);
        expect(std::abs(server.servo-.02)<1e-8,"keyboard left increases raw servo");
        expect(!window.speed->isEnabled()&&!window.steering->isEnabled(),"keyboard disables mouse sliders");
        QEvent blur(QEvent::ApplicationDeactivate);window.eventFilter(&window,&blur);QTest::qWait(350);
        expect(!window.active&&server.erpm==0,"keyboard focus loss stops");
        window.begin();QTest::qWait(250);server.failStatus=true;QTest::qWait(350);
        expect(!window.active&&server.erpm==0&&!window.startButton->isEnabled(),"status failure stops and locks restart");
        server.failStatus=false;QTest::qWait(300);window.controlMode->setCurrentIndex(1);
        expect(!window.startButton->isEnabled(),"unconfigured planning cannot start");
        server.planning=true;QTest::qWait(200);window.begin();QTest::qWait(250);
        expect(window.active&&server.commands.last()["axis"].toInt()==4,"LPP uses independent owner mode");
        window.eventFilter(&window,&up);QTest::qWait(100);
        expect(server.erpm==0&&!window.speed->isEnabled(),"keyboard cannot overwrite planner target");
        window.controlMode->setCurrentIndex(0);QTest::qWait(350);
        expect(!window.active&&server.erpm==0,"LPP to manual switch stops, never auto starts");
        server.maximum=QJsonValue();QTest::qWait(200);
        expect(!window.startButton->isEnabled(),"missing speed limit blocks start");
        server.maximum=2000;QTest::qWait(200);expect(!window.ready(),"excessive speed limit blocks start");
        server.maximum=1850;QTest::qWait(200);
        window.begin();QTest::qWait(200);server.failUpdates=true;QTest::qWait(300);
        expect(!window.active&&server.erpm==0,"rejected UPDATE stops");server.failUpdates=false;
        window.begin();QTest::qWait(200);server.token="restarted-server";QTest::qWait(350);
        expect(!window.active&&server.erpm==0,"server restart token change stops");
        QTest::qWait(200);expect(!window.active,"connection recovery never restarts");
        server.startDelay=500;window.begin();QTest::qWait(100);window.stop();QTest::qWait(650);
        expect(!window.active&&!window.starting&&server.erpm==0,"late START acknowledgement cannot undo STOP");server.startDelay=0;
        window.begin();QTest::qWait(200);server.failStops=true;window.stop();QTest::qWait(300);
        expect(window.stopping&&!window.startButton->isEnabled(),"unacknowledged STOP locks restart");
        server.failStops=false;window.stop();QTest::qWait(350);
        expect(!window.stopping,"explicit STOP retry restores readiness");
        window.begin();QTest::qWait(200);server.statusDelay=1800;QTest::qWait(1900);
        expect(!window.active&&!window.startButton->isEnabled()&&server.erpm==0,"hung status request times out and stops");
        server.statusDelay=0;
        for(const auto& c:server.commands)if(c["action"].toInt()==1)expect(c["motor_erpm"].toDouble()==0,"all starts zero");
        {
            Console sim(false,0,true);QTest::qWait(150);
            expect(sim.ready(),"in-process simulation ready without HTTP server");
            sim.begin();QTest::qWait(100);sim.speed->setValue(800);sim.steering->setValue(800);QTest::qWait(200);
            expect(sim.value("raw_erpm")==800&&std::abs(sim.value("commanded_servo_position")-.2)<1e-9,"simulation inverted servo output");
            expect(sim.value("sim_y")<0,"right GUI input turns vehicle right from forward heading");
            sim.inputMode->setCurrentIndex(1);QTest::qWait(250);sim.begin();QTest::qWait(100);
            sim.eventFilter(&sim,&up);QTest::qWait(200);expect(sim.value("raw_erpm")==50,"simulation keyboard output");
            sim.controlMode->setCurrentIndex(1);QTest::qWait(250);sim.begin();QTest::qWait(600);
            expect(sim.active&&sim.value("raw_erpm")>0&&std::isfinite(sim.value("planning_steering_rad")),"simulation LPP drives synthetic feedback");
            expect(std::hypot(sim.value("sim_x"),sim.value("sim_y"))>0,"simulation moves vehicle pose");
            SimPose straight,leftTurn,rightTurn;for(int i=0;i<10;++i){straight.step(1,0,.1);leftTurn.step(1,.3,.1);rightTurn.step(1,-.3,.1);}
            DriveView camera;camera.observe({{"sim_x",1},{"sim_y",2}});
            expect(camera.cameraCenter()==QPointF(0,0),"fixed camera does not follow movement");
            camera.setFollow(true);expect(camera.cameraCenter()==QPointF(1,2),"follow camera tracks vehicle");
            camera.setFollow(false);camera.observe({{"sim_x",3},{"sim_y",4}});
            expect(camera.cameraCenter()==QPointF(1,2),"switch to fixed holds last viewpoint");
            camera.centerOnVehicle();expect(camera.cameraCenter()==QPointF(3,4),"explicit recenter");
            expect(std::abs(straight.x-1)<1e-9&&straight.y==0,"one second straight movement");
            expect(leftTurn.y>0&&rightTurn.y<0&&std::abs(leftTurn.y+rightTurn.y)<1e-9,"left/right mirrored trajectories");
            const auto render=args.indexOf("--sim-snapshot");if(render>=0&&render+1<args.size()){sim.show();QTest::qWait(100);sim.grab().save(args[render+1]);sim.hide();}
            sim.stop();QTest::qWait(250);expect(!sim.active&&sim.value("raw_erpm")==0,"simulation STOP");
            const double stoppedX=sim.value("sim_x"),stoppedY=sim.value("sim_y");QTest::qWait(150);
            expect(sim.value("sim_x")==stoppedX&&sim.value("sim_y")==stoppedY,"STOP freezes simulated pose");
            sim.simulation->handle("reset-pose",{});sim.controlMode->setCurrentIndex(0);sim.inputMode->setCurrentIndex(0);QTest::qWait(150);
            sim.begin();QTest::qWait(100);sim.steering->setValue(0);sim.speed->setValue(500);QTest::qWait(300);
            expect(sim.value("sim_y")>0&&sim.value("commanded_servo_position")==1,"GUI zero turns vehicle left and raw servo one");
            sim.stop();QTest::qWait(200);
        }
        std::cout<<checks<<" Qt HTTP/input checks, "<<failures<<" failures\n";return failures?1:0;
    }
};
int main(int argc,char** argv){QApplication app(argc,argv);return ConsoleTest::run();}

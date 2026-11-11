#include <QtWidgets>
#include <QtNetwork>
#include <cmath>
#include <functional>
#include <algorithm>
#include "simulation.hpp"
#include "../include/nomad_control_cpp/vehicle_reference.hpp"

// 기존 Pi HTTP 서비스와 통신하는 네이티브 Qt Widgets 클라이언트.
// Qt 창을 열거나 통신을 복구하는 것만으로 START하지 않습니다.
class Console final : public QWidget {
    friend class ConsoleTest;
    QNetworkAccessManager network;
    std::unique_ptr<Simulation> simulation;
    DriveView* driveView=nullptr;
    QElapsedTimer clock;
    QTimer pollTimer, heartbeat;
    QJsonObject status;
    QString token, session;
    quint64 sequence=0, generation=0, polls=0, stopAfter=0;
    bool polling=false, updating=false, starting=false, active=false, stopping=false, stopAck=false, stopBusy=false;
    double receipt=-1e9, offset=0;
    bool clockValid=false,seenRunning=false;
    struct Sample {double at,lower,upper;};
    std::vector<Sample> samples;
    QComboBox *inputMode,*controlMode;
    QPushButton *startButton,*stopButton,*zeroButton;
    QSlider *speed,*steering;
    QLabel *connection,*metrics,*speedValue,*steeringValue,*message,*planner;
    QString origin="http://127.0.0.1:8765";
    using Callback=std::function<void(bool,QJsonObject)>;
    double now() const {return clock.elapsed()/1000.0;}
    bool automatic() const {return controlMode->currentIndex()==1;}
    bool flag(const char* name) const {return status.value(name).toBool();}
    double value(const char* name) const {return status.value(name).toDouble(NAN);}
    bool fresh() const {return clockValid && now()-receipt<=1.5 && flag("connected") && flag("telemetry_fresh");}
    bool contract() const {
        return status["control_units"]=="raw" && value("servo_min")==0 && value("servo_max")==1 &&
            flag("wifi_manual_profile") && flag("commissioning") && value("manual_timeout_s")==2 &&
            value("command_max_age_s")==1.5 && value("browser_freshness_s")==1.5 &&
            value("browser_request_timeout_ms")==1500 && status["motor_control_mode"]=="vesc_speed_pid" &&
            flag("speed_pid_min_erpm_verified") && value("speed_pid_min_erpm")>0 &&
            std::isfinite(value("motor_max_erpm")) && value("motor_max_erpm")>0 && value("motor_max_erpm")<=1850;
    }
    bool ready() const {
        const auto state=status["state"].toString();
        return fresh() && contract() && !flag("preview") && flag("stationary") && flag("driver_ready") &&
            flag("hardware_timeout_verified") && flag("commissioning_setup_acknowledged") &&
            flag("continuous_manual_supported") && !flag("steering_outside_drive_range") &&
            (!flag("stability_enabled") || flag("stability_ready")) &&
            (state=="IDLE" || state=="STOPPED_OUTPUT" || state=="FAULT") &&
            (!automatic() || (flag("planning_supported")&&flag("planning_ready")));
    }
    void request(QString route,QJsonObject body,Callback callback,bool get=false) {
        if(simulation){
            const auto result=simulation->handle(route,body);
            QTimer::singleShot(5,this,[callback,result,get]{callback(get||result["accepted"].toBool(),result);});
            return;
        }
        QNetworkRequest request(QUrl(origin+"/api/"+route));
        request.setRawHeader("Origin",origin.toUtf8());
        request.setRawHeader("X-Nomad-Token",token.toUtf8());
        request.setHeader(QNetworkRequest::ContentTypeHeader,"application/json");
        request.setAttribute(QNetworkRequest::RedirectPolicyAttribute,QNetworkRequest::ManualRedirectPolicy);
        request.setTransferTimeout(1500);
        auto* reply=get?network.get(request):network.post(request,QJsonDocument(body).toJson(QJsonDocument::Compact));
        QTimer::singleShot(1500,reply,[reply]{if(!reply->isFinished())reply->abort();});
        connect(reply,&QNetworkReply::finished,this,[reply,callback,get]{
            const auto doc=QJsonDocument::fromJson(reply->isOpen()?reply->readAll():QByteArray{});
            const auto data=doc.object();
            const bool ok=reply->error()==QNetworkReply::NoError && doc.isObject() &&
                (get || data["accepted"].toBool());
            reply->deleteLater();callback(ok,data);
        });
    }
    QJsonObject command(int action) {
        return {{"action",action},{"session",session},{"sequence",static_cast<double>(++sequence)},
            {"server_time_s",now()+offset},{"axis",automatic()?4:3},{"motor_erpm",automatic()?0:speed->value()},
            {"servo_position",1.0-steering->value()/1000.0},{"duration_s",0}};
    }
    void showState() {
        const bool mouse=inputMode->currentIndex()==0&&!automatic();
        startButton->setEnabled(ready()&&!active&&!starting&&!stopping);
        inputMode->setEnabled(!automatic());
        speed->setEnabled(active&&mouse);steering->setEnabled(active&&mouse);zeroButton->setEnabled(active&&!automatic());
        startButton->setText(active?"제어 중":starting?"시작 중…":automatic()?"LPP 시작":"수동 시작");
        planner->setVisible(automatic());
        const double estimated=simulation?speed->value()/1000.:nomad::referenceSpeedMps(speed->value());
        speedValue->setText(QString("%1 ERPM   ≈ %2 m/s · %3 km/h").arg(speed->value()).arg(estimated,0,'f',3).arg(estimated*3.6,0,'f',2));
        steeringValue->setText(QString("%1  ·  %2").arg(steering->value()/1000.0,0,'f',3).arg(steering->value()<500?"좌회전":steering->value()>500?"우회전":"중앙 입력"));
        connection->setText(!fresh()?"연결 대기":stopping?"정지 확인 중":active?(automatic()?"LPP 자율 제어 중":"수동 조종 중"):starting?"시작 중":
            !flag("commissioning_setup_acknowledged")?"연결됨 · 관찰 전용 · 조종 잠김":ready()?"조종 준비 완료":"시작 조건 미충족");
        planner->setText(!flag("planning_supported")?"LPP: 제어기에 인터페이스 설치 필요":!flag("planning_calibration_verified")?"LPP: m/s·rad 보정값 설정 필요":
            QString("LPP %1 m/s · %2 rad · %3").arg(value("planning_speed_mps"),0,'f',3).arg(value("planning_steering_rad"),0,'f',3).arg(flag("planning_ready")?"수신 중":"명령 대기"));
        metrics->setText(fresh()?QString("현재 회전수\n%1 ERPM\n\n전압   %2 V\n모터 전류   %3 A").arg(value("raw_erpm"),0,'f',0).arg(value("voltage_v"),0,'f',1).arg(value("current_a"),0,'f',2):"현재 회전수\n— ERPM\n\n전압   — V\n모터 전류   — A");
    }
    // 왕복 시간 구간으로 서버 시계 차이를 추정하며 오래된 상태로 제어를 계속하지 않습니다.
    void poll() {
        if(polling)return;
        polling=true;
        const double sent=now();const auto number=++polls;
        request("status",{},[this,sent,number](bool ok,QJsonObject data){
            polling=false;const double received=now();
            if(!ok){clockValid=false;if(active||starting)stop();showState();return;}
            const auto nextToken=data["token"].toString();const double stamp=data["server_time_s"].toDouble(NAN);
            const bool changed=!token.isEmpty()&&token!=nextToken;
            samples.erase(std::remove_if(samples.begin(),samples.end(),[received](const Sample& s){return received-s.at>2;}),samples.end());
            const Sample sample{received,stamp-received,stamp-sent};
            clockValid=std::isfinite(stamp)&&stamp>0&&!changed&&!nextToken.isEmpty();
            for(const auto& s:samples)if(sample.lower>s.upper||s.lower>sample.upper)clockValid=false;
            if(!clockValid)samples.clear();else {samples.push_back(sample);offset=sample.lower;for(const auto& s:samples)offset=std::max(offset,s.lower);}
            token=nextToken;status=data;receipt=received;
            if(driveView)driveView->observe(data);
            if(simulation&&automatic()&&active){
                // 두 슬라이더의 signal로 전체 화면을 중복 갱신하지 않습니다.
                const QSignalBlocker speedBlock(speed),steerBlock(steering);
                speed->setValue(qRound(value("raw_erpm")));steering->setValue(qRound((1-value("commanded_servo_position"))*1000));
            }
            if((active||starting)&&(!fresh()||!contract()||changed))stop();
            if(active&&status["state"]=="RUNNING")seenRunning=true;
            if(active&&seenRunning&&status["state"]!="RUNNING")stop();
            if(stopping&&stopAck&&number>stopAfter&&fresh()&&status["state"]!="RUNNING") {stopping=false;message->setText("출력 해제 완료");}
            const double max=value("motor_max_erpm");if(std::isfinite(max)&&max>=0&&max<=1850)speed->setMaximum(static_cast<int>(max));
            showState();
        },true);
    }
    // 새 세션은 항상 속도 0으로 시작하고, 마지막 서보 명령 위치를 유지합니다.
    void begin() {
        if(!ready()||active||starting||stopping)return;
        speed->setValue(0);const double servo=value("commanded_servo_position");
        if(!std::isfinite(servo)||servo<0||servo>1)return;
        steering->setValue(qRound((1-servo)*1000));session=QUuid::createUuid().toString(QUuid::Id128);sequence=0;
        starting=true;seenRunning=false;const auto attempt=++generation;showState();
        auto sendStart=[this,attempt](bool ok,QJsonObject data){
            if(attempt!=generation)return;
            if(!ok||!fresh()){message->setText(data["reason"].toString("시작 조건을 확인하세요."));stop();return;}
            request("commissioning",command(1),[this,attempt](bool accepted,QJsonObject response){
                if(attempt!=generation)return;
                starting=false;
                if(!accepted||!fresh()){message->setText(response["reason"].toString("시작 실패"));stop();return;}
                active=true;message->setText(inputMode->currentIndex()==1&&!automatic()?"W/S 속도 · A/D 조향 · Esc/Space 정지":"");showState();
            });
        };
        if(status["state"]=="IDLE")sendStart(true,{});else request("reset",{},sendStart);
    }
    // 20Hz 이하, 한 요청씩만 전송해 Wi-Fi 명령 대기열이 누적되지 않게 합니다.
    void update() {
        if(!active||updating)return;
        if(!fresh()){stop();return;}
        updating=true;const auto attempt=generation;
        request("commissioning",command(2),[this,attempt](bool ok,QJsonObject data){
            updating=false;if(attempt!=generation)return;
            if(!ok){message->setText(data["reason"].toString("갱신 실패"));stop();}
        });
    }
    void stop() {
        active=false;starting=false;++generation;speed->setValue(0);stopping=true;showState();
        if(stopBusy)return;
        stopBusy=true;stopAck=false;
        // 세션 STOP이 늦은 START를 무효화한 다음 전역 STOP 응답과 새 상태를 확인합니다.
        auto globalStop=[this](bool sessionOk,QJsonObject){
            request("stop",{},[this,sessionOk](bool ok,QJsonObject){
                stopBusy=false;stopAck=ok&&sessionOk;stopAfter=polls;
                if(!stopAck)message->setText("정지 확인 실패 · STOP으로 다시 확인하세요.");
            });
        };
        if(session.isEmpty())globalStop(true,{});else request("commissioning",command(3),globalStop);
    }
protected:
    bool eventFilter(QObject* object,QEvent* event) override {
        if(event->type()==QEvent::ApplicationDeactivate && inputMode->currentIndex()==1 && (active||starting))stop();
        if(event->type()==QEvent::KeyPress){
            auto* key=static_cast<QKeyEvent*>(event);
            if(key->key()==Qt::Key_Escape||key->key()==Qt::Key_Space){if(!key->isAutoRepeat())stop();return true;}
            if(active&&!automatic()&&inputMode->currentIndex()==1&&!key->modifiers()){
                switch(key->key()){
                    case Qt::Key_W:case Qt::Key_Up:speed->setValue(std::min(speed->maximum(),speed->value()+50));break;
                    case Qt::Key_S:case Qt::Key_Down:speed->setValue(std::max(0,speed->value()-50));break;
                    case Qt::Key_A:case Qt::Key_Left:steering->setValue(std::max(0,steering->value()-20));break;
                    case Qt::Key_D:case Qt::Key_Right:steering->setValue(std::min(1000,steering->value()+20));break;
                    default:return QWidget::eventFilter(object,event);
                }
                showState();return true;
            }
        }
        return QWidget::eventFilter(object,event);
    }
    void closeEvent(QCloseEvent* event) override {
        if(active||starting){event->ignore();stop();QTimer::singleShot(1600,qApp,&QApplication::quit);}
        else event->accept();
    }
public:
    explicit Console(bool preview=false,int testPort=0,bool simulate=false) {
        if(simulate)simulation=std::make_unique<Simulation>();
        if(preview)origin="http://127.0.0.1:8766";
        if(testPort>0)origin="http://127.0.0.1:"+QString::number(testPort);
        network.setProxy(QNetworkProxy::NoProxy);
        clock.start();setWindowTitle("NOMAD Control · Qt");resize(1040,700);
        setStyleSheet("QWidget{background:#101925;color:#eaf1f8;font-size:15px;}QPushButton,QComboBox{padding:11px;background:#293d53;border-radius:7px;}QPushButton:disabled{color:#718096;}QGroupBox{background:#152233;border:1px solid #34465d;border-radius:12px;margin-top:12px;padding:18px;}QGroupBox::title{subcontrol-origin:margin;left:20px;padding:0 5px;}QLabel{padding:4px;background:transparent;}QSlider{background:transparent;min-height:44px;}QSlider::groove:horizontal{height:6px;background:#3b5069;border-radius:3px;}QSlider::sub-page:horizontal{background:#6edac1;border-radius:3px;}QSlider::handle:horizontal{background:#e8fff9;width:24px;margin:-9px 0;border-radius:12px;}QSlider:disabled{opacity:.5;}");
        auto* layout=new QVBoxLayout(this);layout->setContentsMargins(30,24,30,24);layout->setSpacing(14);
        auto* title=new QLabel("NOMAD  /  차량 조종");title->setStyleSheet("font-size:26px;font-weight:bold");layout->addWidget(title);
        if(simulation){title->setText("NOMAD   /   DRIVE LAB");resize(1380,850);}
        if(simulation){setWindowTitle("NOMAD · 모의 테스트 · 실제 차량 연결 없음");auto* banner=new QLabel("모의 모드 · 차량 미연결");banner->setStyleSheet("background:#203c43;color:#89e6d3;padding:8px;border-radius:6px;");layout->addWidget(banner);}
        auto* help=new QPushButton("사용 안내");help->setToolTip("조작 방법과 표시 단위 확인 · 제어 중이면 정지합니다.");
        connect(help,&QPushButton::clicked,this,[this]{if(active||starting)stop();QMessageBox::information(this,"NOMAD 사용 안내",
            "수동: 마우스 슬라이더 또는 W/S 속도, A/D 조향.\n조향 입력 0=좌회전, 0.5=중앙, 1=우회전. 서보 출력은 1−입력값.\nEsc/Space/STOP: 출력 해제. 실제 차량에서는 브레이크가 아닙니다.\n모드 전환 후에는 다시 시작합니다. 키보드 모드는 창을 벗어나면 정지합니다.\n\n고정 화면/차량 따라가기를 선택할 수 있습니다.\n모의 LPP는 예제 명령이며 실제 경로 계획이 아닙니다.\n모의 환산은 1000 ERPM=1m/s, 축간거리0.335m인 후륜 조향 모델입니다.\n실차 속도 표시는 영상 환산이며 실측 속도가 아닙니다.");});
        connection=new QLabel("연결 대기");layout->addWidget(connection);
        auto* modes=new QHBoxLayout;controlMode=new QComboBox;controlMode->addItems({"수동 조종","LPP 자율 제어"});modes->addWidget(new QLabel("제어 모드"));modes->addWidget(controlMode,1);
        inputMode=new QComboBox;inputMode->addItems({"마우스","키보드"});modes->addWidget(new QLabel("입력 방식"));modes->addWidget(inputMode,1);modes->addWidget(help);layout->addLayout(modes);
        auto* columns=new QHBoxLayout;auto* controls=new QGroupBox("조종");auto* left=new QVBoxLayout(controls);
        startButton=new QPushButton("수동 제어 시작");startButton->setStyleSheet("background:#6edac1;color:#10231d;padding:16px");left->addWidget(startButton);
        left->addWidget(new QLabel("속도 목표"));speedValue=new QLabel;left->addWidget(speedValue);speed=new QSlider(Qt::Horizontal);speed->setRange(0,1850);speed->setSingleStep(50);left->addWidget(speed);
        auto* reference=new QLabel("m/s·km/h: 영상 환산");reference->setStyleSheet("font-size:12px;color:#9baec3");left->addWidget(reference);
        if(simulation)reference->setText("모의 속도");
        left->addWidget(new QLabel("조향 입력 · 좌회전 0 ← 중앙 0.5 → 우회전 1"));steeringValue=new QLabel;left->addWidget(steeringValue);steering=new QSlider(Qt::Horizontal);steering->setRange(0,1000);steering->setValue(500);left->addWidget(steering);
        auto* readouts=new QGroupBox("차량 · LPP");auto* right=new QVBoxLayout(readouts);
        metrics=new QLabel;metrics->setStyleSheet("font-size:20px");right->addWidget(metrics);
        planner=new QLabel;planner->setWordWrap(true);right->addWidget(planner);right->addStretch();
        if(simulation){
            driveView=new DriveView;auto* scene=new QGroupBox("주행 화면");auto* sceneLayout=new QVBoxLayout(scene);
            auto* cameraRow=new QHBoxLayout;auto* cameraMode=new QComboBox;cameraMode->addItems({"고정 화면","차량 따라가기"});
            auto* recenter=new QPushButton("차량 위치로 화면 이동");cameraRow->addWidget(cameraMode,1);cameraRow->addWidget(recenter);sceneLayout->addLayout(cameraRow);sceneLayout->addWidget(driveView,1);
            connect(cameraMode,&QComboBox::currentIndexChanged,this,[this](int index){driveView->setFollow(index==1);});
            connect(recenter,&QPushButton::clicked,this,[this]{driveView->centerOnVehicle();});
            auto* resetPose=new QPushButton("정지 · 위치 초기화");sceneLayout->addWidget(resetPose);
            connect(resetPose,&QPushButton::clicked,this,[this]{stop();simulation->handle("reset-pose",{});driveView->clearTrail();});
            auto* side=new QVBoxLayout;side->addWidget(controls,3);side->addWidget(readouts,2);columns->addWidget(scene,3);columns->addLayout(side,2);
            controls->setTitle("조작");readouts->setTitle("상태");
        }else{columns->addWidget(controls,3);columns->addWidget(readouts,2);}
        layout->addLayout(columns,1);
        auto* buttons=new QHBoxLayout;zeroButton=new QPushButton("속도 0");stopButton=new QPushButton("STOP · Esc / Space");stopButton->setStyleSheet("background:#c7354b;padding:16px");buttons->addWidget(zeroButton);buttons->addWidget(stopButton);layout->addLayout(buttons);
        message=new QLabel;message->setWordWrap(true);layout->addWidget(message);
        if(simulation){
            layout->setSpacing(8);layout->setContentsMargins(20,12,20,12);left->setSpacing(5);right->setSpacing(5);
            metrics->setStyleSheet("font-size:15px");
            setStyleSheet(styleSheet()+"QWidget{font-size:13px;}QGroupBox{padding:10px;}QLabel{padding:1px;}QSlider{min-height:28px;}QPushButton,QComboBox{padding:8px;}");
            const auto available=QGuiApplication::primaryScreen()->availableGeometry();resize(std::min(1380,available.width()-40),std::min(850,available.height()-60));
        }
        connect(startButton,&QPushButton::clicked,this,[this]{begin();});connect(stopButton,&QPushButton::clicked,this,[this]{stop();});
        connect(zeroButton,&QPushButton::clicked,this,[this]{speed->setValue(0);update();});
        connect(inputMode,&QComboBox::currentIndexChanged,this,[this]{if(active||starting)stop();showState();});
        connect(controlMode,&QComboBox::currentIndexChanged,this,[this]{if(active||starting)stop();speed->setValue(0);showState();});
        connect(speed,&QSlider::valueChanged,this,[this]{showState();});connect(steering,&QSlider::valueChanged,this,[this]{showState();});
        connect(&pollTimer,&QTimer::timeout,this,[this]{poll();});pollTimer.start(simulation?33:100);
        connect(&heartbeat,&QTimer::timeout,this,[this]{update();});heartbeat.start(50);
        qApp->installEventFilter(this);showState();poll();
    }
};
#ifndef NOMAD_DESKTOP_TEST
int main(int argc,char** argv){
    QApplication app(argc,argv);Console window(app.arguments().contains("--preview"),0,app.arguments().contains("--simulate"));
    // 배포 DLL/Qt 플랫폼 로딩 검사. START 없이 창을 표시하지 않고 종료합니다.
    if(app.arguments().contains("--check"))QTimer::singleShot(250,&app,&QApplication::quit);
    else window.show();
    return app.exec();
}
#endif

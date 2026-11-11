#pragma once
#include <QtWidgets>
#include <cmath>
// 비조향 앞축 기준 평면 모델. 실제 접지/제동/충돌은 재현하지 않습니다.
struct SimPose {
 double x=0,y=0,yaw=0;
 void step(double v,double angle,double dt){
  if(!std::isfinite(v)||!std::isfinite(angle)||!std::isfinite(dt)||dt<=0)return;
  dt=std::min(dt,.2);const double turn=v*std::tan(angle)/.335;
  x+=v*std::cos(yaw+turn*dt/2)*dt;y+=v*std::sin(yaw+turn*dt/2)*dt;
  yaw=std::remainder(yaw+turn*dt,2*3.141592653589793);
 }
};
class DriveView final:public QWidget {
 double x=0,y=0,yaw=0,angle=0,speed=0;QVector<QPointF> trail;
 bool follow=false;QPointF fixedCenter{0,0};
public:
 DriveView(){setMinimumSize(440,360);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);}
 // 시점 변경은 차량 위치/제어 명령을 변경하지 않습니다.
 void setFollow(bool enabled){if(follow&&!enabled)fixedCenter={x,y};follow=enabled;update();}
 void centerOnVehicle(){fixedCenter={x,y};update();}
 QPointF cameraCenter()const{return follow?QPointF(x,y):fixedCenter;}
 void observe(const QJsonObject& s){
  if(!trail.isEmpty()&&x==s["sim_x"].toDouble()&&y==s["sim_y"].toDouble()&&yaw==s["sim_yaw"].toDouble()&&angle==s["sim_angle"].toDouble()&&speed==s["raw_erpm"].toDouble()/1000)return;
  x=s["sim_x"].toDouble();y=s["sim_y"].toDouble();yaw=s["sim_yaw"].toDouble();angle=s["sim_angle"].toDouble();speed=s["raw_erpm"].toDouble()/1000;
  QPointF point(x,y);if(trail.isEmpty()||QLineF(trail.last(),point).length()>.015){trail.append(point);if(trail.size()>2400)trail.remove(0,1);}update();
 }
 void clearTrail(){trail.clear();fixedCenter={0,0};update();}
protected:
 void paintEvent(QPaintEvent*)override{
  QPainter p(this);p.setRenderHint(QPainter::Antialiasing);p.fillRect(rect(),QColor("#0a1420"));
  const double scale=135,cx=width()*.5,cy=height()*.52;
  const auto camera=cameraCenter();
  auto screen=[&](QPointF w){return QPointF(cx+(w.x()-camera.x())*scale,cy-(w.y()-camera.y())*scale);};
  int radius=static_cast<int>(std::max(width(),height())/scale)+2;
  p.setPen(QPen(QColor("#1b3042"),1));
  for(int i=-radius;i<=radius;++i){double gx=std::floor(camera.x())+i,gy=std::floor(camera.y())+i;p.drawLine(screen({gx,camera.y()-radius}),screen({gx,camera.y()+radius}));p.drawLine(screen({camera.x()-radius,gy}),screen({camera.x()+radius,gy}));}
  if(trail.size()>1){QPainterPath path;path.moveTo(screen(trail.first()));for(int i=1;i<trail.size();++i)path.lineTo(screen(trail[i]));p.setPen(QPen(QColor(61,220,187,35),9));p.drawPath(path);p.setPen(QPen(QColor("#4ee4c0"),2));p.drawPath(path);}
  const auto vehicle=screen({x,y});p.save();p.translate(vehicle);p.rotate(-yaw*180/3.141592653589793);
  p.setPen(QPen(QColor("#83f5d9"),2));p.setBrush(QColor("#1d6670"));p.drawRoundedRect(QRectF(-.41*scale,-.14*scale,.49*scale,.28*scale),8,8);
  p.setBrush(QColor("#102c3d"));p.drawRoundedRect(QRectF(-.22*scale,-.1*scale,.16*scale,.2*scale),3,3);
  for(double axle:{0.,-.335})for(double side:{-.155,.155}){p.save();p.translate(axle*scale,side*scale);if(axle<0)p.rotate(angle*180/3.141592653589793);p.setPen(QPen(QColor("#a9bfd2"),1));p.setBrush(QColor("#050a12"));p.drawRoundedRect(QRectF(-9,-4,18,8),2,2);p.restore();}
  p.setPen(QPen(QColor("#eafffa"),2));p.drawLine(8,0,28,0);p.drawLine(28,0,21,-5);p.drawLine(28,0,21,5);p.restore();
  p.setPen(QColor("#8aa6bd"));p.drawText(20,28,follow?"LIVE MOTION  /  차량 따라가기 · 격자 1 m":"LIVE MOTION  /  고정 화면 · 격자 1 m");
  if(!rect().adjusted(0,40,0,-40).contains(vehicle.toPoint())){p.setPen(QColor("#ffc66b"));p.drawText(20,52,"차량이 화면 밖에 있습니다 · ‘차량 위치로 화면 이동’을 누르세요.");}
  p.setPen(QColor("#e9fff8"));
  p.drawText(20,height()-22,QString("%1 m/s    X %2 m    Y %3 m    방향 %4°").arg(speed,0,'f',2).arg(x,0,'f',2).arg(y,0,'f',2).arg(yaw*180/3.141592653589793,0,'f',0));
 }
};

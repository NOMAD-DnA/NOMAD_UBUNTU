'use strict';
// Node 안의 가짜 DOM/HTTP만 사용합니다. 실제 브라우저·Pi·차량에 접속하지 않습니다.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {webcrypto} = require('node:crypto');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
let checks = 0;
function expect(condition, description) { checks++; assert.ok(condition, description); }
const settle = async () => { for (let i=0; i<14; ++i) await Promise.resolve(); };
function deferred() { let resolve,reject; const promise = new Promise((r,j) => { resolve = r;reject=j; }); return {promise, resolve,reject}; }

class Element {
  constructor(tag, attributes='') {
    this.tagName=tag.toUpperCase(); this.listeners={}; this.textContent=''; this.children=[];
    this.value=''; this.min=''; this.max=''; this.step=''; this.hidden=/\bhidden\b/.test(attributes);
    this.disabled=/\bdisabled\b/.test(attributes); this.required=/\brequired\b/.test(attributes);
    for (const m of attributes.matchAll(/([\w-]+)="([^"]*)"/g)) this[m[1]]=m[2];
    const classes=new Set(); this.classList={toggle:(x,on)=>on?classes.add(x):classes.delete(x)};
  }
  addEventListener(type, listener) { (this.listeners[type]??=[]).push(listener); }
  setAttribute(key,value) { this[key]=value; }
  getAttribute(key) { return this[key]===undefined?null:String(this[key]); }
  replaceChildren() { this.children=[]; this.replaceCount=(this.replaceCount||0)+1; }
  append(child) { this.children.push(child); }
  checkValidity() {
    const n=Number(this.value);
    if ((this.required&&this.value==='')||!Number.isFinite(n)) return false;
    if (this.min!==''&&n<Number(this.min)-1e-10) return false;
    if (this.max!==''&&n>Number(this.max)+1e-10) return false;
    if (this.step&&this.step!=='any') {
      const steps=(n-Number(this.min||0))/Number(this.step);
      if (Math.abs(steps-Math.round(steps))>1e-8) return false;
    }
    return true;
  }
}
async function harness(pageProtocol='http:') {
  const elements=new Map();
  for (const m of html.matchAll(/<([a-z]+)\b([^>]*\bid="([^"]+)"[^>]*)>/g)) elements.set(m[3],new Element(m[1],m[2]));
  const document=new Element('document'), window=new Element('window');
  document.hidden=false; document.lookupCount=0;document.getElementById=id=>{document.lookupCount++;return elements.get(id);};
  document.createElement=tag=>new Element(tag);
  let now=0, timerId=0; const timers=new Map(), requests=[],logs=[];
  const transport={statusLatencyMs:0,statusRequests:[],statusRequestTimeouts:[],statusInFlight:0,maxStatusInFlight:0,serverOffsetS:0,
    statusStampOverride:null,commandDeliveryLatencyMs:0,checkCommandAge:false};
  const data={token:'test-only',server_time_s:1000,preview:false,connected:true,control_units:'raw',commissioning:true,state:'IDLE',
    telemetry_fresh:true,stationary:true,driver_ready:true,commissioning_setup_acknowledged:true,hardware_timeout_verified:true,
    continuous_manual_supported:true,continuous_manual_active:false,commanded_servo_position:.55,commanded_motor_erpm:0,
    motor_max_erpm:1850,servo_center:.5,servo_min:.375,servo_max:.625,steering_input_limit_rad:.5,
    motor_control_mode:'vesc_speed_pid',speed_pid_min_erpm:50,speed_pid_min_erpm_verified:true,
    servo_rate_s:.4,raw_erpm:0,current_a:0,voltage_v:16.3,fault_code:0};
  let intercept=null;
  const context=vm.createContext({document,window,location:{protocol:pageProtocol},crypto:webcrypto,Uint8Array,console:{info:value=>logs.push({level:'info',value}),warn:value=>logs.push({level:'warn',value})},performance:{now:()=>now},
    AbortSignal:{timeout:timeoutMs=>({timeoutMs})},
    setTimeout:(fn,delay)=>{const id=++timerId;timers.set(id,{fn,at:now+delay,interval:0});return id;},
    setInterval:(fn,delay)=>{const id=++timerId;timers.set(id,{fn,at:now+delay,interval:delay});return id;},
    clearInterval:id=>timers.delete(id),
    clearTimeout:id=>timers.delete(id),
    fetch:async(url,options={})=>{
      if (!options.method) {
        transport.statusRequests.push(now);transport.statusRequestTimeouts.push(options.signal?.timeoutMs);transport.statusInFlight++;
        transport.maxStatusInFlight=Math.max(transport.maxStatusInFlight,transport.statusInFlight);
        try {
          // 서버에서 생성한 응답이 네트워크를 통과하는 동안 시각과 상태가 낡도록 모델링합니다.
          const generated={...data,server_time_s:transport.statusStampOverride??(1000+now/1000+transport.serverOffsetS)};
          if(transport.statusLatencyMs>0)await new Promise(resolve=>context.setTimeout(resolve,transport.statusLatencyMs));
          return {ok:true,json:async()=>generated};
        } finally {transport.statusInFlight--;}
      }
      const request={route:url.replace('/api/',''),body:JSON.parse(options.body),time:now,timeoutMs:options.signal?.timeoutMs};requests.push(request);
      if (intercept) { const response=intercept(request); if (response) return {ok:true,json:async()=>await response}; }
      if(request.body.action===1||request.body.action===2){
        if(transport.commandDeliveryLatencyMs>0)await new Promise(resolve=>context.setTimeout(resolve,transport.commandDeliveryLatencyMs));
        const age=1000+now/1000+transport.serverOffsetS-request.body.server_time_s;
        request.deliveredAt=now;request.commandAgeS=age;
        const maxAge=data.wifi_manual_profile===true?data.command_max_age_s:.30;
        if(transport.checkCommandAge&&(age<-.05||age>maxAge))return {ok:true,json:async()=>({accepted:false,reason:'simulated stale command'})};
      }
      if (request.route==='reset') data.state='IDLE';
      if (request.route==='stop'||request.body.action===3) { data.state='STOPPED_OUTPUT';data.continuous_manual_active=false; }
      if (request.body.action===1) { data.state='RUNNING';data.continuous_manual_active=true; }
      return {ok:true,json:async()=>({accepted:true,reason:'fake server'})};
    }});
  vm.runInContext(script,context,{filename:'index.html inline script'});await settle();
  function dispatch(target,type,extra={}) {
    const event={target,button:0,repeat:false,defaultPrevented:false,...extra,preventDefault(){this.defaultPrevented=true;}};
    for(const listener of target.listeners[type]||[]) listener(event);
    return event;
  }
  return {elements,requests,data,context,document,window,transport,logs,
    intercept:fn=>{intercept=fn;},
    advanceClock:ms=>{now+=ms;},
    async click(id){const element=elements.get(id);if(!element.disabled)dispatch(element,'click');await settle();},
    async input(id,value){const element=elements.get(id);expect(!element.disabled,id+' input enabled');element.value=String(value);dispatch(element,'input');await settle();},
    async event(target,type,extra){const result=dispatch(target,type,extra);await settle();return result;},
    async run(ms){const end=now+ms;while(now<end){now=Math.min(end,now+10);for(const [id,timer]of [...timers])if(timers.has(id)&&timer.at<=now){if(timer.interval)timer.at+=timer.interval;else timers.delete(id);timer.fn();}await settle();}},
    async render(){vm.runInContext('void poll()',context);await settle();}
  };
}

(async()=>{
  const h=await harness();
  const command=(action)=>h.requests.filter(r=>r.body.action===action);
  expect(h.requests.length===0,'load never starts or resets');
  expect(!h.elements.get('hold').disabled,'fresh stationary controller enables explicit start');
  expect(h.elements.get('steering').disabled,'real raw slider disabled until explicit start');
  await h.click('hold');
  expect(command(1).length===1,'one click sends exactly one START');
  expect(command(1)[0].body.axis===3&&command(1)[0].body.duration_s===0,'continuous combined mode explicitly selected');
  expect(command(1)[0].body.motor_erpm===0&&command(1)[0].body.servo_position===.55,'start speed zero and resumes command candidate');
  const count=command(2).length;
  expect(Number(h.elements.get('steering').min)===-.5&&Number(h.elements.get('steering').max)===.5,'raw UI uses controller-declared ±0.5 rad input range');
  expect(Number(h.elements.get('steeringNumber').value)===.2,'last servo command .55 appears as rounded .2 rad input');
  expect(h.elements.get('degrees').textContent.includes('입력')&&h.elements.get('degrees').textContent.includes('미보정'),'degree display labels uncalibrated input');
  await h.input('steering',.4);
  expect(command(2).length===count,'input following an UPDATE respects the send interval');
  await h.run(50);
  expect(command(2).length===count+1&&command(2).at(-1).body.servo_position===.6,'old +.4 input still maps to .6 within 50ms');
  await h.input('steering',-.4);await h.run(50);
  expect(command(2).at(-1).body.servo_position===.4,'old -.4 input still maps to .4');
  await h.input('steering',-.5);await h.run(50);
  expect(command(2).at(-1).body.servo_position===.375,'expanded negative input endpoint actually expands servo output');
  await h.input('steering',0);await h.run(50);
  expect(command(2).at(-1).body.servo_position===.5,'zero input maps to candidate center servo');
  await h.click('servoHigh');await h.run(50);
  expect(Number(h.elements.get('steeringNumber').value)===.5&&command(2).at(-1).body.servo_position===.625,'positive preset expands angle and bounded servo target');
  expect(Number(h.elements.get('servoTarget').textContent)===.625,'target metric shows converted servo rather than input radians');
  expect(h.elements.get('motorControlHelp').textContent.includes('VESC 내부 PID')&&h.elements.get('motorControlHelp').textContent.includes('50 ERPM'),'verified PID minimum is displayed from controller status');
  const blocked=deferred();let blockedOnce=false;
  h.intercept(r=>{if(r.body.action===2&&!blockedOnce){blockedOnce=true;return blocked.promise;}});
  await h.input('steering',.32);await h.run(50);
  const beforeCoalesce=command(2).length;
  await h.input('speed',500);await h.input('steering',-.4);await h.input('steering',.36);
  expect(command(2).length===beforeCoalesce,'updates serialized during a pending response');
  blocked.resolve({accepted:true});await settle();
  expect(command(2).length===beforeCoalesce,'response completion cannot bypass the 20Hz limit');
  await h.run(50);
  expect(command(2).length===beforeCoalesce+1,'pending inputs coalesce to one request');
  expect(command(2).at(-1).body.motor_erpm===500&&command(2).at(-1).body.servo_position===.59,'coalesced update carries latest speed and steering');
  h.intercept(null);
  const stableChecklist=h.elements.get('checks').replaceCount,lookups=h.document.lookupCount;
  await h.run(4100);
  expect(command(1).length===1&&command(3).length===0,'continuous session remains active beyond three seconds without rearming');
  expect(command(2).length>30,'continuous session sends heartbeats');
  expect(command(2).every((request,index,all)=>index===0||request.time-all[index-1].time>=50),'all UPDATE requests are at most20Hz');
  expect(h.elements.get('checks').replaceCount===stableChecklist&&h.document.lookupCount===lookups,'steady polling reuses DOM references and unchanged checklist');
  await h.input('steering',.1);
  const beforeZero=command(2).length;
  await h.click('zero');await h.run(50);
  expect(command(2).length>beforeZero&&command(2).at(-1).body.motor_erpm===0&&command(3).length===0,'motor zero request keeps the steering session active');
  await h.input('steering',.2); // Leave an UPDATE scheduled to prove STOP bypasses it.
  const updatesAtStop=command(2).length;
  const down=await h.event(h.document,'keydown',{code:'Space',target:h.elements.get('hold')});
  const up=await h.event(h.document,'keyup',{code:'Space',target:h.elements.get('hold')});
  expect(down.defaultPrevented&&up.defaultPrevented,'Space down and up prevent native focused-start activation');
  expect(command(3).length===1&&Number(h.elements.get('speedNumber').value)===0,'Space stops and clears speed');
  expect(command(3)[0].time-command(2).at(-1).time<50,'STOP does not wait behind the UPDATE rate limit');
  await h.run(500);expect(command(1).length===1&&command(2).length===updatesAtStop,'STOP cancels scheduled UPDATE and never automatically restarts');
  await h.click('hold');
  expect(h.requests.some(r=>r.route==='reset')&&command(1).length===2,'explicit restart performs reset then a new START');
  expect(command(1).at(-1).body.motor_erpm===0,'restart never retains previous nonzero motor target');
  const stopsBeforeBlur=command(3).length,updatesBeforeBlur=command(2).length;
  await h.event(h.window,'blur');await h.run(250);
  expect(command(3).length===stopsBeforeBlur&&command(2).length>updatesBeforeBlur,'visible RAW control survives focus moving to another window');
  h.document.hidden=true;await h.event(h.document,'visibilitychange');const stoppedUpdates=command(2).length;
  await h.run(500);expect(command(2).length===stoppedUpdates,'hiding the tab closes heartbeat');
  expect(command(1).length===2,'returning connection does not automatically restart');
  h.document.hidden=false;await h.run(100);

  const late=deferred();h.intercept(r=>r.body.action===1?late.promise:null);
  await h.click('hold');const lateStart=command(1).at(-1);
  h.document.hidden=true;await h.event(h.document,'visibilitychange');
  const tombstone=command(3).at(-1);
  expect(tombstone.body.session===lateStart.body.session&&tombstone.body.sequence>lateStart.body.sequence,'STOP targets in-flight START with higher sequence');
  const beforeLate=command(2).length;late.resolve({accepted:true});await settle();await h.run(500);
  expect(command(2).length===beforeLate,'late START response cannot recreate heartbeat');
  h.document.hidden=false;await h.run(100);h.intercept(null);await h.click('hold');await h.run(100);
  h.data.connected=false;await h.render();const startsBeforeRecovery=command(1).length;
  expect(Number(h.elements.get('speedNumber').value)===0,'disconnect clears displayed speed target');
  h.data.connected=true;h.data.state='IDLE';await h.run(500);
  expect(command(1).length===startsBeforeRecovery,'reconnection never automatically starts');
  const resetWait=deferred();h.data.state='STOPPED_OUTPUT';await h.render();h.intercept(r=>r.route==='reset'?resetWait.promise:null);
  await h.click('hold');h.document.hidden=true;await h.event(h.document,'visibilitychange');resetWait.resolve({accepted:true});await settle();
  expect(command(1).length===startsBeforeRecovery,'tab hiding during reset prevents following START');

  const preview=await harness();preview.data.preview=true;preview.data.connected=false;await preview.render();
  expect(preview.elements.get('hold').disabled,'preview cannot arm');
  expect(!preview.elements.get('steering').disabled,'preview still permits slider inspection');
  await preview.input('steering',.5);expect(preview.requests.length===0,'preview sliders never send commands');
  const invalid=await harness();await invalid.click('hold');
  const beforeInvalid=invalid.requests.filter(r=>r.body.action===2).length;
  await invalid.input('steeringNumber',.501);
  expect(invalid.requests.filter(r=>r.body.action===2).length===beforeInvalid,'input beyond .5 rad never sends an out-of-range target');
  expect(invalid.requests.some(r=>r.body.action===3),'invalid active input closes the session');
  expect(vm.runInContext('Number.isNaN(rawAngleToServo(.501))&&Number.isNaN(rawAngleToServo(NaN))',invalid.context),'mapping rejects invalid values rather than silently clipping');
  expect(vm.runInContext('rawAngleToServo(-.3,{servo_min:.42,servo_center:.51,servo_max:.59,steering_input_limit_rad:.3})===.42&&rawAngleToServo(.3,{servo_min:.42,servo_center:.51,servo_max:.59,steering_input_limit_rad:.3})===.59',invalid.context),'mapping follows asymmetric server limits and declared angle cap');
  const unverified=await harness();unverified.data.speed_pid_min_erpm_verified=false;await unverified.render();await unverified.click('hold');
  expect(unverified.elements.get('hold').disabled&&unverified.requests.length===0,'candidate minimum without firmware verification cannot arm');
  expect(unverified.elements.get('motorControlHelp').textContent.includes('확인 전'),'unverified PID metadata is not presented as a confirmed firmware value');
  unverified.data.speed_pid_min_erpm_verified=true;delete unverified.data.steering_input_limit_rad;await unverified.render();await unverified.click('hold');
  expect(unverified.requests.length===0,'old controller without explicit angle metadata cannot arm mismatched GUI');

  const observed=await harness();
  expect(observed.elements.get('observedStartHelp').textContent.includes('관찰값 없음')&&!observed.elements.get('hold').disabled,'optional observation absent is unknown and does not change the arming contract');
  observed.data.observed_smooth_start_erpm=550;
  observed.data.observed_smooth_start_note='공중 시험 · 사용자 관찰 · 실제 주행 최저속도 미보정';
  await observed.render();await observed.click('hold');
  expect(observed.elements.get('motorControlHelp').textContent.includes('펌웨어 최소 설정 50 ERPM')&&!observed.elements.get('motorControlHelp').textContent.includes('550'),'verified firmware minimum is distinct from operator observation');
  expect(observed.elements.get('observedStartHelp').textContent.includes('약 550 ERPM')&&observed.elements.get('observedStartHelp').textContent.includes('공중 시험 · 사용자 관찰 · 실제 주행 최저속도 미보정'),'approximate observed target includes its uncalibrated test conditions');
  expect(observed.requests.find(r=>r.body.action===1).body.motor_erpm===0&&observed.elements.get('lowSpeedWarning').hidden,'observation never boosts START above zero');
  for(const target of [50,500]){
    await observed.input('speed',target);await observed.run(50);
    expect(!observed.elements.get('lowSpeedWarning').hidden&&html.includes('이 범위는 멈추거나 끊겨 회전할 수 있습니다.'),'positive target below observed start warns about irregular rotation');
    expect(observed.requests.filter(r=>r.body.action===2).at(-1).body.motor_erpm===target&&Number(observed.elements.get('speedNumber').value)===target,'low target is transmitted without clamping or boost');
  }
  await observed.input('speed',550);await observed.run(50);
  expect(observed.elements.get('lowSpeedWarning').hidden,'observed threshold is not treated as below itself');
  await observed.input('speed',600);await observed.run(50);
  expect(observed.elements.get('lowSpeedWarning').hidden,'target above observed threshold has no low-speed warning');
  await observed.input('speed',500);await observed.click('zero');await observed.run(50);
  expect(observed.elements.get('lowSpeedWarning').hidden&&observed.requests.filter(r=>r.body.action===2).at(-1).body.motor_erpm===0,'zero clears warning and remains zero with steering session active');
  await observed.input('speed',500);await observed.click('stop');
  expect(observed.elements.get('lowSpeedWarning').hidden&&Number(observed.elements.get('speedNumber').value)===0&&observed.requests.some(r=>r.body.action===3),'STOP stays immediate and clears displayed warning');
  observed.data.observed_smooth_start_erpm=0;await observed.render();
  expect(observed.elements.get('observedStartHelp').textContent.includes('관찰값 없음'),'unset zero observation is displayed as unknown');
  observed.data.observed_smooth_start_erpm=550;observed.data.speed_pid_min_erpm_verified=false;await observed.render();
  expect(observed.elements.get('hold').disabled&&observed.elements.get('motorControlHelp').textContent.includes('확인 전'),'operator observation cannot verify the firmware minimum');

  const expanded=await harness();expanded.data.steering_input_limit_rad=.55;expanded.data.servo_min=.3625;expanded.data.servo_max=.6375;
  await expanded.render();await expanded.click('hold');await expanded.input('steering',.55);await expanded.run(50);
  expect(Number(expanded.elements.get('steering').max)===.55&&expanded.requests.filter(r=>r.body.action===2).at(-1).body.servo_position===.6375,'a new controller-declared range needs no GUI hard-coded limit');
  expect(Number(expanded.elements.get('servoTarget').textContent)===.6375&&expanded.elements.get('steeringHelp').textContent.includes('0.3625'),'expanded endpoints retain four-decimal servo display precision');

  const held=await harness();Object.assign(held.data,{control_units:'calibrated',calibration_verified:true,speed_min:0,speed_max:.4,steering_min:-.2,steering_max:.2});
  await held.render();await held.event(held.elements.get('hold'),'pointerdown');await held.event(held.window,'blur');
  expect(held.requests.some(r=>r.route==='manual'&&r.body.action===1)&&held.requests.some(r=>r.route==='manual'&&r.body.action===3),'non-RAW press-and-hold mode still stops on window blur');
  const exited=await harness();await exited.click('hold');await exited.event(exited.window,'pagehide');
  const exitedUpdates=exited.requests.filter(r=>r.body.action===2).length;await exited.run(200);
  expect(exited.requests.some(r=>r.body.action===3)&&exited.requests.filter(r=>r.body.action===2).length===exitedUpdates,'leaving the page still closes RAW control and heartbeat');

  const delayed=await harness();delayed.transport.statusLatencyMs=80;await delayed.click('hold');
  let inFlight=0,maxInFlight=0;
  delayed.intercept(request=>{
    if(request.body.action!==2)return null;
    inFlight++;maxInFlight=Math.max(maxInFlight,inFlight);
    return new Promise(resolve=>delayed.context.setTimeout(()=>{inFlight--;resolve({accepted:true});},200));
  });
  await delayed.run(2100);
  const slowUpdates=delayed.requests.filter(r=>r.body.action===2);
  expect(maxInFlight===1&&slowUpdates.length>=11,'200ms UPDATE responses keep one request in flight and continue without extra heartbeat idle time');
  expect(slowUpdates.every((r,i,a)=>i===0||r.time-a[i-1].time<=200),'pending heartbeat sends immediately when a slow UPDATE response completes');
  expect(!delayed.requests.some(r=>r.body.action===3),'healthy delayed transport does not end a continuous session');
  expect(delayed.transport.maxStatusInFlight===1&&delayed.transport.statusRequests.every((t,i,a)=>i===0||t-a[i-1]===100),'80ms status HTTP time is included in the 100ms cadence without overlapping requests');
  await delayed.input('speed',500);const updateCount=slowUpdates.length;
  await delayed.click('stop');
  expect(inFlight===1&&delayed.requests.some(r=>r.body.action===3)&&delayed.requests.some(r=>r.route==='stop'),'session and global STOP bypass an unresolved UPDATE');
  await delayed.run(500);
  expect(delayed.requests.filter(r=>r.body.action===2).length===updateCount,'late accepted UPDATE cannot flush a pending target after STOP');

  const stale=await harness();await stale.click('hold');stale.transport.statusLatencyMs=429;
  await stale.run(450);
  expect(stale.requests.some(r=>r.body.action===3),'429ms status delay still closes the session after the 300ms freshness limit');
  const staleCount=stale.requests.filter(r=>r.body.action===2).length;
  await stale.run(600);
  expect(stale.requests.filter(r=>r.body.action===2).length===staleCount&&stale.requests.filter(r=>r.body.action===1).length===1,'status recovery does not restore heartbeats or automatically restart');

  const stopping=await harness();await stopping.click('hold');
  const globalStop=deferred();stopping.intercept(r=>r.route==='stop'?globalStop.promise:null);
  await stopping.click('stop');await stopping.run(100);
  expect(stopping.data.state==='STOPPED_OUTPUT'&&stopping.elements.get('hold').disabled,'session STOP and fresh stationary status cannot rearm while global STOP is pending');
  await stopping.click('hold');
  expect(stopping.requests.filter(r=>r.body.action===1).length===1,'START cannot race a pending global STOP');
  await stopping.click('quickStop');
  expect(stopping.requests.filter(r=>r.route==='stop').length===1,'repeated STOP coalesces one global RPC so newer ACK cannot overtake an unresolved older STOP');
  stopping.transport.statusLatencyMs=200;
  await stopping.run(100); // A status request starts before the global acknowledgement.
  globalStop.resolve({accepted:true});await settle();
  expect(stopping.elements.get('hold').disabled,'latest coalesced guard remains locked immediately after the shared STOP acknowledgement');
  await stopping.run(200);
  expect(stopping.elements.get('hold').disabled,'a status response whose request preceded STOP acknowledgement cannot unlock START');
  stopping.transport.statusLatencyMs=0;
  await stopping.run(250);
  expect(!stopping.elements.get('hold').disabled,'fresh stopped status requested after the successful STOP acknowledgement permits an explicit restart');
  expect(stopping.requests.filter(r=>r.body.action===1).length===1,'successful STOP confirmation never starts automatically');

  const stopFailure=await harness();await stopFailure.click('hold');
  const failedStop=deferred();stopFailure.intercept(r=>r.route==='stop'?failedStop.promise:null);
  await stopFailure.click('stop');failedStop.reject(Error('simulated timeout'));await settle();await stopFailure.run(300);
  expect(stopFailure.elements.get('hold').disabled&&stopFailure.elements.get('startHelp').textContent.includes('STOP을 다시'),'timed-out global STOP remains locked after fresh stationary status and explains retry');
  expect(!stopFailure.elements.get('stop').disabled&&!stopFailure.elements.get('quickStop').disabled,'STOP retry controls remain available after timeout');
  await stopFailure.click('hold');
  expect(stopFailure.requests.filter(r=>r.body.action===1).length===1&&!stopFailure.requests.some(r=>r.route==='reset'),'STOP failure does not allow START or automatically reset a fault');
  const retryStop=deferred();stopFailure.intercept(r=>r.route==='stop'?retryStop.promise:null);
  await stopFailure.click('quickStop');await stopFailure.run(100);
  expect(stopFailure.requests.filter(r=>r.route==='stop').length===2&&stopFailure.elements.get('hold').disabled,'retry sends a new global STOP but keeps START locked');
  retryStop.resolve({accepted:true});await settle();
  expect(stopFailure.elements.get('hold').disabled,'successful retry acknowledgement alone does not unlock START');
  await stopFailure.run(100);
  expect(!stopFailure.elements.get('hold').disabled,'successful explicit retry and its subsequent fresh status clear the failed STOP guard');
  const refused=deferred();stopFailure.intercept(r=>r.route==='stop'?refused.promise:null);
  await stopFailure.click('stop');refused.resolve({accepted:false,reason:'rejected stop'});await settle();await stopFailure.run(100);
  expect(stopFailure.elements.get('hold').disabled&&stopFailure.elements.get('startHelp').textContent.includes('정지 확인 실패'),'rejected STOP is not treated as successful confirmation');
  const diagnostics=delayed.logs.map(entry=>JSON.parse(entry.value.replace('[NOMAD timing] ','')));
  expect(diagnostics.some(entry=>entry.route==='commissioning'&&entry.action===2&&entry.elapsed_ms===200&&Number.isFinite(entry.status_age_on_send_ms)&&Number.isFinite(entry.last_poll_rtt_ms)&&Number.isFinite(entry.server_time_sent)),'timing diagnostics capture response latency and cached status age without changing requests');
  expect(stopFailure.logs.some(entry=>entry.level==='warn'&&entry.value.includes('simulated timeout'))&&stopFailure.logs.some(entry=>entry.level==='warn'&&entry.value.includes('rejected stop')),'failed and rejected requests always log timing warnings');
  expect(diagnostics.every(entry=>!('token'in entry)&&!('session'in entry)&&!('motor_erpm'in entry)&&!('servo_position'in entry))&&!delayed.logs.some(entry=>entry.value.includes('test-only')),'diagnostics exclude credentials, session IDs, and command targets');
  const quiet=await harness();await quiet.click('hold');await quiet.run(2000);
  expect(quiet.logs.filter(entry=>entry.level==='info').length===20&&!quiet.logs.some(entry=>entry.level==='warn'),'healthy fast traffic logs only the first ten POSTs and ten polls');

  const stamp=await harness();vm.runInContext('lastPollRttMs=90',stamp.context);stamp.advanceClock(200);
  expect(vm.runInContext('commandServerTime()',stamp.context)===1000.2,'receipt anchor advances by 200ms of monotonic time without adding network RTT');
  await stamp.click('hold');
  expect(stamp.requests.find(r=>r.body.action===1).body.server_time_s===1000.2&&stamp.requests.find(r=>r.body.action===2).body.server_time_s===1000.2,'START and immediate UPDATE use the conservative timestamp');
  await stamp.render();
  expect(vm.runInContext('serverTime===1000.2&&lastPoll===200&&commandServerTime()===1000.2',stamp.context),'a compatible fresh status advances freshness independently of the clock history');
  stamp.advanceClock(50);await stamp.input('steering',.1);
  expect(stamp.requests.filter(r=>r.body.action===2).at(-1).body.server_time_s===1000.25,'later UPDATE uses elapsed time from the newest poll anchor');
  stamp.advanceClock(251);
  expect(vm.runInContext('!freshBrowser()',stamp.context),'extrapolation does not extend the 300ms status freshness limit');
  assert.throws(()=>vm.runInContext('commandServerTime()',stamp.context),/최신 서버 시각/);checks++;
  const stampUpdates=stamp.requests.filter(r=>r.body.action===2).length;await stamp.input('steering',.2);
  expect(stamp.requests.filter(r=>r.body.action===2).length===stampUpdates&&stamp.requests.some(r=>r.body.action===3),'stale status prevents extrapolated UPDATE and closes the session');
  await stamp.render();stamp.document.hidden=true;
  assert.throws(()=>vm.runInContext('commandServerTime()',stamp.context),/최신 서버 시각/);checks++;
  stamp.document.hidden=false;vm.runInContext('lastPoll=performance.now()+1',stamp.context);
  expect(vm.runInContext('!freshBrowser()',stamp.context),'a backward monotonic clock or future anchor is not fresh');
  assert.throws(()=>vm.runInContext('commandServerTime()',stamp.context),/최신 서버 시각/);checks++;
  const expiredClock=await harness();expiredClock.advanceClock(1001);
  expect(vm.runInContext('Number.isNaN(conservativeServerTime())',expiredClock.context),'clock samples expire after one second even before another response arrives');
  const futureClock=await harness();vm.runInContext('clockLastReceipt=performance.now()+1',futureClock.context);
  assert.throws(()=>vm.runInContext('commandServerTime()',futureClock.context),/최신 서버 시각/);checks++;
  expect(vm.runInContext('!observeServerClock(20,10,1000,token)&&clockSamples.length===0',futureClock.context),'negative RTT invalidates every clock sample');

  // 왕복 지연을 서버 시각에 더하지 않아도, 빠른 최근 표본이 느린 응답의 하행 지연을 누적하지 않게 합니다.
  const downlink=await harness();downlink.transport.checkCommandAge=true;await downlink.click('hold');await downlink.run(100);
  downlink.transport.statusLatencyMs=200;downlink.transport.commandDeliveryLatencyMs=150;
  await downlink.run(700);
  const delivered=downlink.requests.filter(r=>r.body.action===2&&r.deliveredAt>=250);
  expect(delivered.length>=4&&delivered.every(r=>r.commandAgeS>=0&&r.commandAgeS<.151),'200ms status downlink plus150ms command delivery does not add the old downlink delay to command age');
  expect(downlink.data.state==='RUNNING'&&!downlink.requests.some(r=>r.body.action===3),'transient delayed responses retain continuous control with fresh status and a recent compatible anchor');
  expect(vm.runInContext('conservativeServerTime()-(serverTime+(performance.now()-lastPoll)/1000)>.19',downlink.context),'a slow status response cannot replace the recent tighter conservative bound');
  expect(delivered.every(r=>r.body.server_time_s<=1000+r.time/1000),'conservative command timestamps never exceed the simulated server clock at dispatch');
  downlink.transport.statusLatencyMs=0;await downlink.run(500);
  expect(downlink.data.state==='RUNNING'&&vm.runInContext('clockSamples.every(s=>performance.now()-s.receivedAt<=1000)',downlink.context),'fast status recovery refreshes the bounded history without restarting the session');

  for(const shift of [-2,2]){
    const changedClock=await harness();await changedClock.click('hold');changedClock.advanceClock(100);
    changedClock.transport.serverOffsetS=shift;await changedClock.render();
    expect(changedClock.requests.some(r=>r.body.action===3)&&changedClock.elements.get('hold').disabled,'incompatible server clock step closes active control and cannot immediately rearm');
    assert.throws(()=>vm.runInContext('commandServerTime()',changedClock.context),/최신 서버 시각/);checks++;
    await changedClock.run(200);
    expect(changedClock.requests.filter(r=>r.body.action===1).length===1&&changedClock.data.state==='STOPPED_OUTPUT','new clock samples never automatically restore a stopped session');
  }
  for(const invalidStamp of [NaN,Infinity,0,-1]){
    const invalidClock=await harness();await invalidClock.click('hold');invalidClock.transport.statusStampOverride=invalidStamp;await invalidClock.render();
    expect(invalidClock.requests.some(r=>r.body.action===3)&&invalidClock.elements.get('hold').disabled,'nonfinite or nonpositive server clock invalidates the anchor and closes control');
  }
  const replayedClock=await harness();await replayedClock.click('hold');replayedClock.advanceClock(200);
  replayedClock.transport.statusStampOverride=1000;await replayedClock.render();
  expect(replayedClock.requests.some(r=>r.body.action===3)&&vm.runInContext('clockSamples.length===0',replayedClock.context),'a replayed old response outside the compatible time interval closes the session');
  replayedClock.transport.statusStampOverride=null;await replayedClock.run(200);
  expect(replayedClock.requests.filter(r=>r.body.action===1).length===1,'clock recovery after a replay does not send another START');
  const restartedServer=await harness();await restartedServer.click('hold');restartedServer.data.token='replacement-test-only';await restartedServer.render();
  expect(restartedServer.requests.some(r=>r.body.action===3)&&restartedServer.elements.get('hold').disabled,'server token change resets the clock and active session');
  await restartedServer.run(200);
  expect(restartedServer.requests.filter(r=>r.body.action===1).length===1,'new server identity requires a later explicit START');

  const wifiProfile={wifi_manual_profile:true,manual_timeout_s:2,command_max_age_s:1.5,browser_freshness_s:1.5,browser_request_timeout_ms:1500};
  const wifi=await harness();Object.assign(wifi.data,wifiProfile);await wifi.render();await wifi.click('hold');
  expect(!wifi.elements.get('networkProfileHelp').hidden&&wifi.elements.get('networkProfileHelp').textContent.includes('마지막 명령 시각부터 약 2초')&&wifi.elements.get('networkProfileHelp').textContent.includes('STOP 전달도 지연'),'Wi-Fi bench profile visibly explains the source-time stop bound and delayed STOP delivery');
  wifi.transport.statusLatencyMs=1200;let wifiInFlight=0,wifiMaxInFlight=0;
  wifi.intercept(request=>{
    if(request.body.action!==2)return null;
    request.commandAgeS=1000+request.time/1000-request.body.server_time_s;
    wifiInFlight++;wifiMaxInFlight=Math.max(wifiMaxInFlight,wifiInFlight);
    return new Promise(resolve=>wifi.context.setTimeout(()=>{wifiInFlight--;resolve({accepted:request.commandAgeS>=-.05&&request.commandAgeS<=1.5});},1200));
  });
  await wifi.run(6100);
  const wifiUpdates=wifi.requests.filter(r=>r.body.action===2&&Number.isFinite(r.commandAgeS));
  expect(wifi.data.state==='RUNNING'&&!wifi.requests.some(r=>r.body.action===3)&&wifiUpdates.length>=5,'explicit Wi-Fi bench profile tolerates1200ms status/UPDATE response jitter without a total-duration stop');
  expect(wifiMaxInFlight===1&&wifiUpdates.every(r=>r.commandAgeS>=0&&r.commandAgeS<=1.5),'Wi-Fi profile keeps bounded one-in-flight updates and conservative command age within its explicit limit');
  expect(wifiUpdates.every(r=>r.timeoutMs===1500)&&wifi.transport.statusRequestTimeouts.slice(2).every(ms=>ms===1500),'only the validated Wi-Fi profile requests1500ms HTTP deadlines');
  await wifi.click('stop');const wifiCount=wifi.requests.filter(r=>r.body.action===2).length;await wifi.run(1500);
  expect(wifi.requests.some(r=>r.route==='stop')&&wifi.requests.filter(r=>r.body.action===2).length===wifiCount&&wifi.requests.filter(r=>r.body.action===1).length===1,'STOP bypasses the delayed Wi-Fi UPDATE and late replies never resume control');

  const wifiStale=await harness();Object.assign(wifiStale.data,wifiProfile);await wifiStale.render();await wifiStale.click('hold');
  wifiStale.transport.statusLatencyMs=1700;await wifiStale.run(1600);
  expect(wifiStale.requests.some(r=>r.body.action===3)&&Number(wifiStale.elements.get('speedNumber').value)===0,'Wi-Fi status beyond1500ms closes the active session');
  await wifiStale.run(2000);
  expect(wifiStale.requests.filter(r=>r.body.action===1).length===1,'over-budget Wi-Fi status recovery never automatically restarts');
  const wifiTimeout=await harness();Object.assign(wifiTimeout.data,wifiProfile);await wifiTimeout.render();await wifiTimeout.click('hold');
  wifiTimeout.intercept(request=>request.body.action===2?new Promise((resolve,reject)=>wifiTimeout.context.setTimeout(()=>reject(Error('simulated1500ms deadline')),1500)):null);
  await wifiTimeout.run(1600);
  expect(wifiTimeout.requests.some(r=>r.body.action===3),'Wi-Fi UPDATE request failure still requests STOP');
  await wifiTimeout.run(500);
  expect(wifiTimeout.requests.filter(r=>r.body.action===1).length===1,'request deadline recovery cannot restart Wi-Fi control');
  const wifiExpiry=await harness();Object.assign(wifiExpiry.data,wifiProfile);await wifiExpiry.render();wifiExpiry.advanceClock(1501);
  expect(vm.runInContext('!freshBrowser()&&Number.isFinite(conservativeServerTime())',wifiExpiry.context),'Wi-Fi freshness expires at1500ms independently of its two-second clock history');
  wifiExpiry.advanceClock(500);
  expect(vm.runInContext('Number.isNaN(conservativeServerTime())',wifiExpiry.context),'Wi-Fi clock anchor cannot outlive two seconds');

  const invalidProfiles=[{wifi_manual_profile:'true'},{manual_timeout_s:3},{command_max_age_s:2},{browser_freshness_s:2},
    {browser_request_timeout_ms:2000},{browser_request_timeout_ms:undefined},{commissioning:false},{control_units:'calibrated'},
    {wifi_manual_profile:false},{continuous_manual_supported:false}];
  for(const invalidFields of invalidProfiles){
    const malformed=await harness();Object.assign(malformed.data,wifiProfile,invalidFields);await malformed.render();await malformed.click('hold');
    expect(malformed.elements.get('hold').disabled&&!malformed.requests.some(r=>r.body.action===1)&&malformed.elements.get('networkProfileHelp').textContent.includes('올바르지'),'malformed, incomplete or non-RAW Wi-Fi profile cannot arm');
  }
  const invalidDuring=await harness();Object.assign(invalidDuring.data,wifiProfile);await invalidDuring.render();await invalidDuring.click('hold');
  invalidDuring.data.browser_freshness_s=9;await invalidDuring.render();
  expect(invalidDuring.requests.some(r=>r.body.action===3)&&invalidDuring.elements.get('hold').disabled,'invalid runtime Wi-Fi profile closes existing control instead of silently extending deadlines');
  const strictDefaults=await harness();await strictDefaults.click('hold');await strictDefaults.run(200);
  expect(strictDefaults.requests.every(r=>r.timeoutMs===600)&&strictDefaults.transport.statusRequestTimeouts.every(ms=>ms===600)&&vm.runInContext('controlTiming().freshMs===300',strictDefaults.context),'strict default HTTP600ms and freshness300ms remain unchanged');
  const filePage=await harness('file:');
  expect(!filePage.elements.get('offlineHelp').hidden&&filePage.elements.get('hold').disabled&&filePage.transport.statusRequests.length===0,
    'direct HTML file explains launcher requirement without polling or enabling controls');
  const viewMode=await harness();viewMode.data.commissioning_setup_acknowledged=false;await viewMode.render();await viewMode.click('hold');
  expect(viewMode.elements.get('readinessTitle').textContent==='상태 보기 · 조작 잠김'&&viewMode.elements.get('hold').disabled&&!viewMode.requests.some(r=>r.body.action===1),
    'view-only mode explains control lock and cannot start');
  const stabilityBlocked=await harness();Object.assign(stabilityBlocked.data,{stability_enabled:true,stability_ready:false,stability_reason:'IMU missing'});await stabilityBlocked.render();await stabilityBlocked.click('hold');
  expect(stabilityBlocked.elements.get('hold').disabled&&stabilityBlocked.elements.get('readinessDetail').textContent==='IMU missing'&&!stabilityBlocked.requests.some(r=>r.body.action===1),
    'stability failure gives actionable reason and keeps START blocked');
  const cal=await harness();cal.data.steering_records_supported=true;cal.data.steering_command_recording_supported=true;cal.data.max_duration_s=3;await cal.render();await cal.click('calMode');
  expect(vm.runInContext('calibrationMode',cal.context)&&cal.elements.get('speed').disabled&&cal.elements.get('hold').disabled,'calibration locks ordinary driving controls');
  cal.elements.get('speedNumber').value='1000';await cal.click('calPlus');await cal.run(100);
  const calMotion=cal.requests.filter(r=>r.body.action===1||r.body.action===2);
  expect(calMotion.length>0&&calMotion.every(r=>r.body.axis===2&&r.body.motor_erpm===0&&r.body.duration_s===3&&r.body.record_steering_command),'calibration always uses STEERING axis with zero motor and automatic command recording');
  expect(cal.elements.get('calSave').disabled&&cal.elements.get('calMode').disabled,'cannot save measurements or switch to drive during motion');
  cal.data.state='RUNNING';await cal.render();cal.data.commanded_servo_position=.555;cal.data.state='STOPPED_OUTPUT';await cal.render();
  expect(cal.elements.get('calLeft').disabled&&!cal.elements.get('calSave').disabled,'angles are optional and disabled by default');
  await cal.input('calNote','moves smoothly');await cal.click('calSave');
  const observation=cal.requests.find(r=>r.route==='steering-records'&&r.body.operation==='save');
  expect(observation.body.kind==='observation'&&observation.body.left_deg===null&&observation.body.measured_confirmed===false,'memo saves unknown angle as null without measurement confirmation');
  cal.elements.get('calMeasured').checked=true;await cal.event(cal.elements.get('calMeasured'),'input');
  await cal.input('calLeft',12.3);await cal.input('calRight',10.1);cal.elements.get('calConfirmed').checked=true;
  await cal.event(cal.elements.get('calConfirmed'),'input');await cal.click('calSave');
  const saved=cal.requests.find(r=>r.route==='steering-records'&&r.body.operation==='save'&&r.body.kind==='measured');
  expect(saved&&saved.body.servo_command===.555&&saved.body.left_deg===12.3&&saved.body.right_deg===10.1&&saved.body.measured_confirmed,'save pairs measured wheel angles with received command position');
  cal.data.commanded_servo_position=cal.data.servo_max;await cal.render();const before=cal.requests.filter(r=>r.body.action===1).length;await cal.click('calPlus');
  expect(cal.requests.filter(r=>r.body.action===1).length===before,'calibration never expands existing servo limit');
  await cal.input('calLeft',12.3);await cal.input('calRight',10.1);cal.elements.get('calConfirmed').checked=true;
  await cal.event(cal.elements.get('calConfirmed'),'input');cal.data.commanded_servo_position-=.005;await cal.render();
  expect(cal.elements.get('calSave').disabled&&!cal.elements.get('calConfirmed').checked&&cal.elements.get('calLeft').value==='',
    'a changed servo command invalidates the old measured angles');
  await cal.input('calLeft',12.3);await cal.input('calRight',10.1);cal.elements.get('calConfirmed').checked=true;
  await cal.event(cal.elements.get('calConfirmed'),'input');cal.data.connected=false;await cal.render();
  expect(cal.elements.get('calSave').disabled&&!cal.elements.get('calConfirmed').checked&&cal.elements.get('calRight').value==='',
    'disconnect invalidates measurement confirmation and prevents stale saves');
  const range=await harness();Object.assign(range.data,{steering_records_supported:true,steering_command_recording_supported:true,
    steering_range_config_supported:true,steering_test_servo_min:.375,steering_test_servo_max:.625,steering_test_span_max:.5});
  await range.render();await range.click('calMode');
  range.intercept(r=>{if(r.route==='steering-range'){range.data.steering_test_servo_min=.5-r.body.span;range.data.steering_test_servo_max=.5+r.body.span;
    return Promise.resolve({accepted:true,servo_min:range.data.steering_test_servo_min,servo_max:range.data.steering_test_servo_max});}});
  await range.input('calRange',.3);
  expect(!range.requests.some(r=>r.route==='steering-range'||r.body.action===1),'dragging maximum slider sends neither limit update nor START');
  await range.click('calRangeApply');await range.render();
  expect(range.requests.some(r=>r.route==='steering-range'&&r.body.span===.3)&&!range.requests.some(r=>r.body.action===1||r.body.action===2),'applying maximum only configures range without movement');
  range.data.commanded_servo_position=.7;range.data.steering_outside_drive_range=true;await range.render();await range.click('calPlus');
  expect(range.requests.some(r=>r.body.action===1&&r.body.axis===2&&r.body.motor_erpm===0&&Math.abs(r.body.servo_position-.705)<1e-8),'steering-only steps use expanded range while motor remains zero');
  expect(range.elements.get('calRange').disabled&&range.elements.get('calRangeApply').disabled,'maximum editing locked during steering motion');
  range.data.state='RUNNING';await range.render();range.data.state='STOPPED_OUTPUT';await range.render();await range.click('calMode');await range.render();
  expect(range.elements.get('hold').disabled,'normal driving stays disabled outside ordinary steering range');
  console.log(checks+' continuous GUI checks, 0 failures (fake DOM/HTTP only)');
  const direct=await harness();Object.assign(direct.data,{servo_min:0,servo_max:1,commanded_servo_position:.5});
  await direct.render();await direct.click('hold');
  expect(direct.elements.get('steering').min===0&&direct.elements.get('steering').max===1,'existing steering slider exposes 0 to 1');
  for(const value of [0,.5,1]){await direct.input('steering',value);await direct.run(50);
    expect(direct.requests.filter(r=>r.body.action===2).at(-1).body.servo_position===value,'direct servo value transmitted without rad conversion');}
  expect(direct.requests.filter(r=>r.body.action===1||r.body.action===2).every(r=>r.body.motor_erpm===0),'steering-only input leaves speed at zero');
  direct.elements.get('inputMode').value='keyboard';await direct.event(direct.elements.get('inputMode'),'change');await direct.run(150);
  expect(direct.requests.some(r=>r.route==='stop'),'changing input mode ends current session');
  await direct.render();await direct.click('hold');await direct.run(100);
  const beforeKey=direct.requests.filter(r=>r.body.action===2).length;
  await direct.event(direct.document,'keydown',{code:'KeyW'});await direct.run(50);
  expect(direct.requests.filter(r=>r.body.action===2).at(-1).body.motor_erpm===50,'keyboard W requests 50 ERPM through existing session');
  expect(direct.elements.get('speed').disabled&&direct.elements.get('steering').disabled,'mouse sliders disabled in keyboard mode');
  const priorServo=Number(direct.elements.get('steeringNumber').value);
  await direct.event(direct.document,'keydown',{code:'KeyA'});await direct.run(50);
  expect(Math.abs(direct.requests.filter(r=>r.body.action===2).at(-1).body.servo_position-Math.max(0,priorServo-.02))<1e-8,'keyboard A changes normalized servo without rad conversion');
  await direct.event(direct.window,'blur');
  expect(direct.requests.filter(r=>r.body.action===3).length>0&&Number(direct.elements.get('speedNumber').value)===0,'keyboard focus loss releases session and clears target');
  console.log('Direct servo and keyboard checks passed');
})().catch(error=>{console.error(error);process.exitCode=1;});

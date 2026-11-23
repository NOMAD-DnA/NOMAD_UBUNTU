"""RTK UI/relay/recording contracts; no hardware or network access."""
import os
from pathlib import Path
import sys
from unittest import mock
import time
import pytest
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
os.environ.setdefault('NOMAD_VIEWER_STATE_DIR','/tmp/nomad-rtk-ui-test')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets
from rtk_panel import RtkPanel,public_status,summary
from relay import owns_stream
import monitor

@pytest.fixture(scope='module')
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def status(enabled=True):
    return dict(session='test',enabled=enabled,state='RECEIVING' if enabled else 'OFF',
        reason='',receiver=dict(solution=5,satellites=19,hacc=.02),receiver_age_s=.1,
        receiving=enabled,rx_age_s=.2,tx_age_s=.1,used_age_s=None,rx_bytes=1024,tx_bytes=1024,
        events=[dict(id=1,t=1000.,code='AUTH_OK'),dict(id=2,t=1001.,code='RTK_FIXED')],
        mountpoints=['VRS-RTCM32'],working=enabled)


def test_no_credentials_in_public_telemetry():
    assert 'password' not in public_status(dict(status(),password='PRIVATE',config={'password':'PRIVATE'}))
    assert 'PRIVATE' not in str(public_status(dict(status(),password='PRIVATE',config={'password':'PRIVATE'})))
    data=status();data['receiver']['password']='PRIVATE';data['events'][0]['password']='PRIVATE'
    assert 'PRIVATE' not in str(public_status(data))


@pytest.mark.parametrize('patch',[{'receiver':[]},{'rx_age_s':'x'},{'events':[{'t':float('nan')}]},
                                {'mountpoints':['x\npassword']},{'enabled':'true'}])
def test_invalid_status_rejected(patch):
    with pytest.raises(ValueError):public_status(dict(status(),**patch))


def test_readonly_cannot_connect_or_send(app):
    panel=RtkPanel('user@example',readonly=True)
    with mock.patch('rtk_panel.RtkClient',side_effect=AssertionError('No replay SSH')):
        panel.open();panel.connect_board();panel.command('start');panel.command('stop')
    assert panel.client is None
    assert not panel.start.isEnabled() and not panel.connect.isEnabled()
    panel.close()


def test_receiver_fix_is_not_faked_by_off_button(app):
    panel=RtkPanel('user@example')
    panel.update_status(status(False),0.)
    assert '꺼짐' in panel.label.text() and 'RTK FIXED' in panel.label.text()
    panel.update_status(status(True),5.)
    assert '오래됨' in panel.label.text() and not panel.start.isEnabled()
    panel.close()


def test_logs_rewind_and_settings_mask_password(app):
    panel=RtkPanel('user@example',readonly=True)
    assert panel.fields['password'].echoMode()==QtWidgets.QLineEdit.Password
    panel.update_status(status(),0.)
    assert 'RTK FIXED' in panel.log.toPlainText()
    early=status();early['events']=early['events'][:1]
    panel.update_status(early,0.)
    assert 'RTK FIXED' not in panel.log.toPlainText()
    panel.update_status({},None)
    assert panel.log.toPlainText()==''
    panel.close()


def test_rtk_uses_pi_stream_and_preserves_drive_status(app):
    with mock.patch.object(sys,'argv',['monitor','--no-ssh']):args=monitor.parse_args()
    window=monitor.MonitorWindow(args)
    try:
        window.on_packet('pi',dict(kind='status',source='localization',state='READY',t=time.time()))
        before=dict(window.statuses)
        event=dict(kind='telemetry',source='rtk',t=time.time(),rtk=status())
        assert owns_stream('pi',event) and not owns_stream('pi4',event)
        assert window.on_packet('pi',event)
        window.refresh()
        assert '수신 중' in window.rtk_panel.label.text()
        event['t']=time.time();event['rtk']=dict(status(),state='RETRYING',reason='NETWORK_ERROR',receiving=False)
        assert window.on_packet('pi',event)
        assert window.statuses==before and window.planning_client is None
        bad=dict(event,rtk={'receiver':[]})
        assert not window.on_packet('pi',bad)
    finally:window.close()

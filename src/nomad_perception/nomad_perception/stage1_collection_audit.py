"""Local producer ledgers and content audits; no DDS metadata substitutes for this file."""
from __future__ import annotations
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np


def payload_sha256(message):
    return hashlib.sha256(bytes(message.data)).hexdigest()


def calibration_identity(info):
    from .semantic_input import stamp_ns
    return {'stamp_ns': stamp_ns(info), 'frame_id': info.header.frame_id,
            'width': int(info.width), 'height': int(info.height),
            'k': list(info.k), 'd': list(info.d),'r':list(getattr(info,'r',[])),'p':list(getattr(info,'p',[])),
            'calibration_sha256': hashlib.sha256(json.dumps({'k':list(info.k),'d':list(info.d),
                'width':int(info.width),'height':int(info.height)},sort_keys=True).encode()).hexdigest()}


class PublisherLedger:
    def __init__(self, directory, experiment_id, run_id, publisher_id, settings):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory/'sources').mkdir()
        self.stream = (self.directory/'publisher_ledger.jsonl').open('x')
        self.identity = {'experiment_id':experiment_id,'run_id':run_id,'publisher_id':publisher_id}
        self.sequence=0; self.armed=False; self.end_stamp=None; self.finished=False
        self.instrumentation_ns=0; self.last_slot=None; self.selected_stamps=set()
        self.event('header',schema='stage1-producer-ledger-v1',settings=settings,
            upstream_camera_generation='unverified; ledger begins at ROS bridge adapter subscription',
            selection_rule='first received depth exposure in each floor(capture_stamp/interval_ns) slot, declared before pairing')

    def event(self, kind, **values):
        started=time.perf_counter_ns()
        row={'event':kind,'monotonic_ns':time.monotonic_ns(),**self.identity,**values}
        self.stream.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n');self.stream.flush()
        self.instrumentation_ns += time.perf_counter_ns()-started

    def arm(self, stamp):
        if self.armed:raise ValueError('Observation producer already armed')
        self.armed=True;self.start_stamp=int(stamp);self.last_slot=None
        self.event('audit_start',capture_stamp_ns=self.start_stamp,next_publish_sequence=self.sequence)

    def source_capture(self, capture, cloud, pixel_offset):
        from .semantic_input import image_pixels,stamp_ns
        started=time.perf_counter_ns()
        relative=f'sources/{self.sequence:09d}.npz'
        metadata={'depth_encoding':capture['depth'].encoding,'labels_encoding':capture['labels'].encoding,
            'depth_stamp_ns':stamp_ns(capture['depth']),'labels_stamp_ns':stamp_ns(capture['labels']),
            'depth_frame':capture['depth'].header.frame_id,'labels_frame':capture['labels'].header.frame_id,
            'depth_info':calibration_identity(capture['depth_info']),
            'labels_info':calibration_identity(capture['labels_info']),
            'pixel_center_offset_px':pixel_offset}
        destination=self.directory/relative
        with destination.open('xb') as stream:
            np.savez_compressed(stream,depth=image_pixels(capture['depth']),
                labels=image_pixels(capture['labels']),metadata=np.array(json.dumps(metadata)))
        result={'source_file':relative,'source_sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}
        self.instrumentation_ns+=time.perf_counter_ns()-started
        return result

    def published(self, cloud, source):
        from .semantic_input import stamp_ns
        self.event('published',sequence=self.sequence,stamp_ns=stamp_ns(cloud),
            point_count=int(cloud.width*cloud.height),message_data_bytes=len(cloud.data),
            payload_sha256=payload_sha256(cloud),**source)
        self.sequence+=1

    def finalize(self, end_stamp, pending, counts):
        if self.finished:return
        self.end_stamp=int(end_stamp)
        self.event('audit_end',capture_stamp_ns=self.end_stamp,final_publish_sequence=self.sequence-1,
            published_count=self.sequence,pending_exposure_items=pending,final_counts=counts,
            instrumentation_cpu_boundary='producer trace and selected raw archival cost outside candidate CPU',
            instrumentation_wall_ns=self.instrumentation_ns)
        self.stream.flush();os.fsync(self.stream.fileno());self.stream.close();self.finished=True
        path=self.directory/'finalized.json'
        with path.open('x') as stream:
            json.dump({'finalized':True,**self.identity,'end_stamp_ns':self.end_stamp,
                'next_sequence':self.sequence,'pending_exposure_items':pending},stream,indent=2)
            stream.flush();os.fsync(stream.fileno())


def read_ledger(directory):
    path=Path(directory)/'publisher_ledger.jsonl'
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    starts=[r for r in rows if r['event']=='audit_start'];ends=[r for r in rows if r['event']=='audit_end']
    if len(starts)!=1 or len(ends)!=1 or rows[-1]['event']!='audit_end':
        raise ValueError('Producer audit start/end missing, duplicated or nonfinal')
    published=[r for r in rows if r['event']=='published']
    sequences=[r['sequence'] for r in published]
    if sequences!=list(range(len(published))) or ends[0]['final_publish_sequence']!=len(published)-1:
        raise ValueError('Producer publication sequence is incomplete')
    begin,end=starts[0]['capture_stamp_ns'],ends[0]['capture_stamp_ns']
    selected=[r for r in published if begin<=r['stamp_ns']<=end]
    if len({r['stamp_ns'] for r in selected})!=len(selected):
        raise ValueError('Publication stamps collide within run; no unique bag mapping')
    return rows,selected,starts[0],ends[0]


def reconstruct_local_source(directory,row):
    from types import SimpleNamespace as NS
    from .semantic_input import project_semantic_points,semantic_cloud_payload
    path=Path(directory)/row['source_file']
    if hashlib.sha256(path.read_bytes()).hexdigest()!=row['source_sha256']:
        raise ValueError('Selected original raw capture file hash mismatch')
    with np.load(path,allow_pickle=False) as data:
        depth=data['depth'];labels=data['labels'];meta=json.loads(str(data['metadata']))
    def header(stamp,frame):return NS(stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9),frame_id=frame)
    def image(array,encoding,stamp,frame):
        return NS(header=header(stamp,frame),width=array.shape[1],height=array.shape[0],
            encoding=encoding,is_bigendian=int(array.dtype.byteorder=='>'),step=len(array[0].tobytes()),data=array.tobytes())
    def info(values):return NS(header=header(values['stamp_ns'],values['frame_id']),
        width=values['width'],height=values['height'],k=values['k'],d=values['d'],r=values.get('r',[]),p=values.get('p',[]))
    xyz,classes=project_semantic_points(image(depth,meta['depth_encoding'],meta['depth_stamp_ns'],meta['depth_frame']),
        image(labels,meta['labels_encoding'],meta['labels_stamp_ns'],meta['labels_frame']),
        info(meta['depth_info']),info(meta['labels_info']),pixel_center_offset_px=meta['pixel_center_offset_px'])
    digest=hashlib.sha256(semantic_cloud_payload(xyz,classes)).hexdigest()
    if digest!=row['payload_sha256']:raise ValueError('Raw capture reconstruction differs from emitted PointCloud2 payload')
    return xyz,classes,meta


def audit_bag_collection(bag,ledger_directory,archive=None):
    from .stage1_record import _bag_reader
    from .semantic_input import stamp_ns
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows,published,start,end=read_ledger(ledger_directory)
    expected={r['stamp_ns']:r for r in published};observed=[];topics={}
    reader=_bag_reader(bag);types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic,data,receive_stamp=reader.read_next();stat=topics.setdefault(topic,{'count':0,'serialized_bytes':0,
            'receive_stamp_min_ns':receive_stamp,'receive_stamp_max_ns':receive_stamp})
        stat['count']+=1;stat['serialized_bytes']+=len(data);stat['receive_stamp_max_ns']=receive_stamp
        if topic!='/nomad/semantic_points':continue
        cloud=deserialize_message(data,types[topic]);stamp=stamp_ns(cloud)
        if start['capture_stamp_ns']<=stamp<=end['capture_stamp_ns']:
            observed.append({'stamp_ns':stamp,'payload_sha256':payload_sha256(cloud)})
    stamps=[r['stamp_ns'] for r in observed];counts=Counter(stamps)
    missing=sorted(set(expected)-set(stamps));extra=sorted(set(stamps)-set(expected))
    duplicate=[stamp for stamp,count in counts.items() if count>1]
    mismatched=[r['stamp_ns'] for r in observed if r['stamp_ns'] in expected and
        r['payload_sha256']!=expected[r['stamp_ns']]['payload_sha256']]
    reconstruction_errors=[]
    for row in published:
        try:reconstruct_local_source(ledger_directory,row)
        except Exception as error:reconstruction_errors.append({'stamp_ns':row['stamp_ns'],'error':str(error)})
    collection_pass=not(missing or extra or duplicate or mismatched) and stamps==sorted(stamps) and bool(expected)
    reconstructed=not reconstruction_errors and bool(expected)
    scoped=[r for r in rows if start['capture_stamp_ns']<=r.get('stamp_ns',r.get('capture_stamp_ns',-1))<=end['capture_stamp_ns']]
    selected_depth={r['stamp_ns'] for r in scoped if r['event']=='depth_selected'}
    invalid={r['stamp_ns'] for r in scoped if r['event']=='selected_projection_invalid'}
    empty={r['stamp_ns'] for r in scoped if r['event']=='selected_no_valid_depth'}
    unresolved=sorted(selected_depth-set(expected)-invalid-empty)
    result={'schema':'stage1-collection-audit-v1','audit_start':start,'audit_end':end,
        'verified_start_boundary':'semantic publisher local publish-call ledger',
        'camera_to_bridge':{'state':'unverified','reason':'Gazebo camera generated ID set is not instrumented'},
        'semantic_publisher_to_bag':{'state':'complete' if collection_pass else 'incomplete',
            'published':len(expected),'saved':len(stamps),'missing_stamps':missing,'extra_stamps':extra,
            'duplicate_stamps':duplicate,'payload_mismatch_stamps':mismatched,'order_verified':stamps==sorted(stamps)},
        'raw_frame_reconstructability':{'state':'complete' if reconstructed else 'incomplete',
            'checked':len(expected),'errors':reconstruction_errors,
            'preserved_scope':'all selected original depth/GT/calibration exposures for emitted semantic clouds in producer sources; bag also contains raw stream subset',
            'upstream_whole_raw_stream_completeness':'unverified'},
        'topic_storage':topics,'rgb_saved_in_bag':'/oak/rgb/image_raw' in topics,
        'producer_lifetime_event_counts':dict(Counter(r['event'] for r in rows)),
        'stage_event_counts_in_audit_window':dict(Counter(r['event'] for r in scoped)),
        'selected_depth_outcomes':{'selected_depth_stamps':sorted(selected_depth),'published_stamps':sorted(expected),
            'invalid_projection_stamps':sorted(invalid),'no_valid_depth_stamps':sorted(empty),
            'unresolved_selected_capture_stamps':unresolved,
            'unresolved_reason':'Inspect exact stream exclusion records; no automatic DDS cause assigned'},
        'complete_within_declared_contract':collection_pass and reconstructed,
        'intervals':{'basis':'emitted semantic capture stamps inside declared audit interval'}}
    intervals=np.diff(stamps)/1e9
    if len(intervals):result['intervals'].update(median_s=float(np.median(intervals)),p95_s=float(np.percentile(intervals,95)),
        max_gap_s=float(intervals.max()),max_gap_start_stamp_ns=stamps[int(intervals.argmax())])
    if archive is not None:
        from .traversability_record import ObservationArchive
        archive_stamps=[];archive_payloads_valid=True
        for frame in ObservationArchive(archive):
            archive_stamps.append(frame.stamp_ns)
            archive_payloads_valid &= frame.stamp_ns in expected and frame.metadata.get('source_payload_sha256')==expected[frame.stamp_ns]['payload_sha256']
        archive_pass=archive_stamps==[r['stamp_ns'] for r in observed] and archive_payloads_valid
        result['bag_to_archive']={'state':'complete' if archive_pass else 'incomplete',
            'bag_inputs':len(observed),'archive_inputs':len(archive_stamps),'ordered_ID_set_equal':archive_pass}
        result['complete_within_declared_contract'] &= archive_pass
    return result

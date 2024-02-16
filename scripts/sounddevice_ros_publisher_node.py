#!/usr/bin/env python

from __future__ import print_function

"""Create a recording with arbitrary duration.

PySoundFile (https://github.com/bastibe/PySoundFile/) has to be installed!

"""
import argparse
import tempfile
import sys
if sys.version_info[0] < 3:
    from Queue import Queue
else:
    from queue import Queue

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    '-l', '--list-devices', action='store_true',
    help='show list of audio devices and exit')
parser.add_argument(
    '-d', '--device', type=int_or_str,
    help='input device (numeric ID or substring)')
parser.add_argument(
    '-r', '--samplerate', type=int, help='sampling rate')
parser.add_argument(
    '-c', '--channels', type=int, default=1, help='number of input channels')
parser.add_argument(
    'filename', nargs='?', metavar='FILENAME',
    help='audio file to store recording to')
parser.add_argument(
    '-t', '--subtype', type=str, help='sound file subtype (e.g. "PCM_24")')
parser.add_argument(
    '-s', '--save_file', type=bool, default=False, help='flag to save to audio file')
parser.add_argument(
    '-n', '--num', type=str, help='audio number')
args = parser.parse_args()

try:
    import rospy
    import sounddevice as sd
    import soundfile as sf
    from sounddevice_ros.msg import AudioInfo, AudioData
    import numpy  # Make sure NumPy is loaded before it is used in the callback
    assert numpy  # avoid "imported but unused" message (W0611)

    if args.list_devices:
        print(sd.query_devices())
        parser.exit(0)
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, 'input')
        # soundfile expects an int, sounddevice provides a float:
        args.samplerate = int(device_info['default_samplerate'])
    if args.save_file:
        if args.filename is None:
            args.filename = tempfile.mktemp(prefix='delme_rec_unlimited_',
                                            suffix='.wav', dir='')
    if args.num is None:
        args.num = ''
        
    q = Queue()
    audio_info_pub = rospy.Publisher('/audio'+args.num+'_info', AudioInfo, queue_size=10)
    audio_pub = rospy.Publisher('/audio'+args.num, AudioData, queue_size=10)
    rospy.init_node('sounddevice_ros_publisher'+args.num)

    def callback(indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        q.put(indata.copy())

    if args.save_file:
        # Make sure the file is opened before recording anything:
        with sf.SoundFile(args.filename, mode='x', samplerate=args.samplerate,
                          channels=args.channels, subtype=args.subtype) as file:
            with sd.InputStream(samplerate=args.samplerate, device=args.device,
                                channels=args.channels, callback=callback):

                while not rospy.is_shutdown():
                    audio_data = q.get()
                    file.write(audio_data)
                    ai = AudioInfo()
                    ai.header.stamp = rospy.Time.now()
                    ai.num_channels = args.channels
                    ai.sample_rate = args.samplerate
                    if args.subtype is not None:
                        ai.subtype = args.subtype
                    audio_info_pub.publish(ai)
                    ad = AudioData()
                    ad.header.stamp = rospy.Time.now()
                    ad.data = audio_data.flatten()
                    audio_pub.publish(ad)
    else:
        with sd.InputStream(samplerate=args.samplerate, device=args.device,
                                channels=args.channels, callback=callback):
            while not rospy.is_shutdown():
                audio_data = q.get()
                ai = AudioInfo()
                ai.header.stamp = rospy.Time.now()
                ai.num_channels = args.channels
                ai.sample_rate = args.samplerate
                if args.subtype is not None:
                    ai.subtype = args.subtype
                audio_info_pub.publish(ai)
                ad = AudioData()
                ad.header.stamp = rospy.Time.now()
                ad.data = audio_data.flatten()
                audio_pub.publish(ad)

except KeyboardInterrupt:
    print('\nRecording finished: ' + repr(args.filename))
    parser.exit(0)
except Exception as e:
    parser.exit(type(e).__name__ + ': ' + str(e))

{
  'includes': [
    'src/talk/build/common.gypi',
  ],
  'targets': [
    {
      'target_name': 'bare_executable',
      'type': 'executable',
      'sources': [
        'bare.cc',
      ],
      'dependencies': [
        '<(DEPTH)/third_party/jsoncpp/jsoncpp.gyp:jsoncpp',
        '<(DEPTH)/third_party/libyuv/libyuv.gyp:libyuv',
        '<(DEPTH)/talk/libjingle.gyp:libjingle_peerconnection',
      ],
    },  # target peerconnection_client
  ],
}

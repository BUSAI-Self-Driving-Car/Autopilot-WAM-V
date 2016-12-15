include apt

# http://www.puppetcookbook.com/posts/set-global-exec-path.html
Exec { path => [ "/bin/", "/sbin/" , "/usr/bin/", "/usr/sbin/" ] }

node robotx {
  $archs = {
    'native'   => {'flags'       => ['-march=native', '-mtune=native', ],
                   'params'      => ['-m64', ],
                   'environment' => {'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  },
    'nuc'      => {'flags'       => ['-march=broadwell', '-mtune=broadwell', '-mabm', '-madx', '-maes', '-mavx', '-mavx2', '-mbmi', '-mbmi2', '-mcx16', '-mf16c', '-mfma', '-mfsgsbase', '-mfxsr', '-mlzcnt', '-mmmx', '-mmovbe', '-mpclmul', '-mpopcnt', '-mprfchw', '-mrdrnd', '-mrdseed', '-msahf', '-msse', '-msse2', '-msse3', '-msse4', '-msse4.1', '-msse4.2', '-mssse3', '-mxsave', '-mxsaveopt', ],
                   'params'      => ['-m64', '--param l1-cache-size=32', '--param l1-cache-line-size=64', '--param l2-cache-size=4096', ],
                   'environment' => {'TARGET' => 'HASWELL', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  },
    'e38'      => {'flags'       => ['-march=silvermont', '-mtune=silvermont', '-maes', '-mcx16', '-mfxsr', '-mmmx', '-mmovbe', '-mpclmul', '-mpopcnt', '-mprfchw', '-mrdrnd', '-msahf', '-msse', '-msse2', '-msse3', '-msse4', '-msse4.1', '-msse4.2', '-mssse3', ],
                   'params'      => ['-m64', '--param l1-cache-size=24', '--param l1-cache-line-size=64', '--param l2-cache-size=1024', ],
                   'environment' => {'TARGET' => 'NEHALEM', 'USE_THREAD' => '1', 'BINARY' => '64', 'NUM_THREADS' => '2', 'AUDIO' => 'PORTAUDIO', 'LDFLAGS' => '-m64', },
                  }
  }

  # Make sure the necessary installer prerequisites are satisfied.
  class { 'installer::prerequisites' :
    archs => $archs,
  }

  # We need dev tools to use the installer
  class {'dev_tools': } -> Installer <| |>

  # List all of the archives that need to be downloaded along with any other associated parameters (creates, requires, etc).
  $archives = {
    'protobuf'     => {'url'         => 'https://github.com/google/protobuf/releases/download/v3.1.0/protobuf-cpp-3.1.0.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib', '--with-protoc=PROTOC_PATH', ],
                                          'nuc' => [ '--with-zlib', '--with-protoc=PROTOC_PATH',  ],
                                          'e38' => [ '--with-zlib', '--with-protoc=PROTOC_PATH',  ], },
                       'require'     => [ Class['protobuf'], Installer['zlib'], ],
                       'prebuild'    => 'make distclean',
                       'postbuild'   => 'rm PREFIX/lib/libprotoc* && rm PREFIX/bin/protoc',
                       'method'      => 'autotools', },

    'zlib'         => {'url'         => 'http://zlib.net/zlib-1.2.8.tar.gz',
                       'creates'     => 'lib/libz.a',
                       'method'      => 'cmake',},

    'bzip2-static' => {'url'         => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz',
                       'creates'     => 'lib/libbz2.a',
                       'method'      => 'make',},

    'bzip2-shared' => {'url'         => 'http://www.bzip.org/1.0.6/bzip2-1.0.6.tar.gz',
                       'creates'     => 'lib/libbz2.so',
                       'args'        => { 'native'   => [ '-f Makefile-libbz2_so', ],
                                          'nuc' => [ '-f Makefile-libbz2_so', ],
                                          'e38' => [ '-f Makefile-libbz2_so', ], },
                       'method'      => 'make',
                       'require'     => [ Exec['correct_bzip2-shared_Makefile_10'], ],},

    'xml2'         => {'url'         => 'http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz',
                       'args'        => { 'native'   => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'nuc' => [ '--with-zlib=ZLIB_PATH', '--without-python', ],
                                          'e38' => [ '--with-zlib=ZLIB_PATH', '--without-python', ], },
                       'method'      => 'autotools',},

    'nuclear'      => {'url'         => 'https://github.com/Fastcode/NUClear/archive/release/1.0.tar.gz',
                       'args'        => { 'native'   => [ '-DBUILD_TESTS=OFF', ],
                                          'nuc' => [ '-DBUILD_TESTS=OFF', ],
                                          'e38' => [ '-DBUILD_TESTS=OFF', ], },
                       'method'      => 'cmake',},
    # NOTE: OpenBLAS CMake support is experimental and only supports x86 at the moment.
    'openblas'     => {'url'         => 'https://github.com/xianyi/OpenBLAS/archive/v0.2.18.tar.gz',
                       'method'      => 'make',},

    'tcmalloc'     => {'url'         => 'https://github.com/gperftools/gperftools/releases/download/gperftools-2.5/gperftools-2.5.tar.gz',
                       'args'        => { 'native'   => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'nuc' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ],
                                          'e38' => [ '--with-tcmalloc-pagesize=64', '--enable-minimal', ], },
                       'creates'     => 'lib/libtcmalloc_minimal.a',
                       'method'      => 'autotools',},

    'yaml-cpp'     => {'url'         => 'https://github.com/jbeder/yaml-cpp/archive/master.tar.gz',
                       'args'        => { 'native'   => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'nuc' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ],
                                          'e38' => [ '-DYAML_CPP_BUILD_CONTRIB=OFF', '-DYAML_CPP_BUILD_TOOLS=OFF', ], },
                       'method'      => 'cmake',},

    'fftw3'        => {'url'         => 'http://www.fftw.org/fftw-3.3.4.tar.gz',
                       'args'        => { 'native'   => [ '--disable-fortran', '--enable-shared', ],
                                          'nuc' => [ '--disable-fortran', '--enable-shared', ],
                                          'e38' => [ '--disable-fortran', '--enable-shared', ], },
                       'method'      => 'autotools',},

    'jpeg'         => {'url'         => 'http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.2/libjpeg-turbo-1.4.2.tar.gz',
                       'args'        => { 'native'   => [ 'CCASFLAGS="-f elf64"', ],
                                          'nuc' => [ 'CCASFLAGS="-f elf64"', ],
                                          'e38' => [ 'CCASFLAGS="-f elf64"', ], },
                       'method'      => 'autotools',},

    'cppformat'    => {'url'         => 'https://github.com/cppformat/cppformat/archive/2.0.0.tar.gz',
                       'method'      => 'cmake', },

    'ffmpeg'       => {'url'         => 'http://ffmpeg.org/releases/ffmpeg-3.2.2.tar.bz2',
                       'creates'     => 'lib/libavformat.so',
                       'args'        => { 'native'   => [ '--enable-gpl', '--enable-shared', '--enable-libx264', ],
                                          'nuc' => [ '--enable-gpl', '--enable-shared', '--enable-libx264', ],
                                          'e38' => [ '--enable-gpl', '--enable-shared', '--enable-libx264', ], },
                       'method'      => 'autotools', },

    'portaudio'    => {'url'         => 'http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc' => [ '', ],
                                          'e38' => [ '', ], },
                       'method'      => 'autotools',},

    'rtaudio'      => {'url'         => 'http://www.music.mcgill.ca/~gary/rtaudio/release/rtaudio-4.1.1.tar.gz',
                       'args'        => { 'native'   => [ '', ],
                                          'nuc' => [ '', ],
                                          'e38' => [ '', ], },
                       'method'      => 'autotools',},

    'muparserx'    => {'url'         => 'https://github.com/beltoforion/muparserx/archive/v4.0.4.tar.gz',
                       'method'      => 'cmake',},

    'eigen3'       => {'url'         => 'http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz',
                       'creates'     => 'include/eigen3/Eigen/Eigen',
                       'method'      => 'cmake',},

    'boost'        => {'url'         => 'http://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz',
                       'args'        => { 'native'   => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'nuc' => [ 'address-model=64', 'architecture=x86', 'link=static', ],
                                          'e38' => [ 'address-model=64', 'architecture=x86', 'link=static', ], },
                       'method'      => 'boost',
                       'creates'     => 'src/boost/build_complete',
                       'postbuild'   => 'touch build_complete',
                       'require'     => [ Installer['zlib'], Installer['bzip2-shared'], Installer['bzip2-static'], ],},

    'espeak'       => {'url'         => 'http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip',
                       'src_dir'     => 'espeak-1.48.04-source/src',
                       'prebuild'    => 'cp portaudio19.h portaudio.h',
                       'method'      => 'make',
                       'require'     => [ File_line['correct_espeak_Makefile'], Installer['portaudio'] ]},

    'pybind11'     => {'url'         => 'https://github.com/pybind/pybind11/archive/master.tar.gz',
                       'creates'     => 'include/pybind11/pybind11.h',
                       'method'      => 'cmake',
                       'args'        => { 'native'   => [ '-DPYBIND11_TEST=OFF', ],
                                          'nuc' => [ '-DPYBIND11_TEST=OFF', ],
                                          'e38' => [ '-DPYBIND11_TEST=OFF', ], }, },

    'fswatch'      => {'url'         => 'https://github.com/emcrisostomo/fswatch/releases/download/1.9.3/fswatch-1.9.3.tar.gz',
                       'creates'     => 'lib/libfswatch.a',
                       'method'      => 'autotools', },

    'x264'         => {'url'         => 'ftp://ftp.videolan.org/pub/x264/snapshots/last_x264.tar.bz2',
                       'creates'     => 'lib/libx264.so',
                       'method'      => 'autotools',
                       'args'        => { 'native'   => [ '--enable-static', '--enable-shared', ],
                                          'nuc' => [ '--enable-static', '--enable-shared', ],
                                          'e38' => [ '--enable-static', '--enable-shared', ], }, },

    'qpOASES'      => {'url'         => 'https://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.0.zip',
                       'creates'     => 'lib/libqpOASES.a',
                       'src_dir'     => 'qpOASES-3.2.0',
                       'method'      => 'cmake',
                       'args'        => { 'native'   => [ '-DCMAKE_CXX_FLAGS="-fPIC"', ],
                                          'nuc' => [ '-DCMAKE_CXX_FLAGS="-fPIC"', ],
                                          'e38' => [ '-DCMAKE_CXX_FLAGS="-fPIC"', ], }, },



  }


  # Correct CXXFLAGS definition in eSpeak Makefile to firstly append to CXXFLAGS and to allow narrowing conversions to be treated as warnings.
  file_line { 'correct_espeak_Makefile':
    path    => '/robotx/toolchain/src/espeak/espeak-1.48.04-source/src/Makefile',
    match   => '^CXXFLAGS=-O2',
    line    => 'CXXFLAGS += -Wno-error=narrowing',
    ensure  => present,
    require => [ Archive['espeak'], ],
  }

  # Append an install target to the bzip2-shared Makefile.
  file_line { 'correct_bzip2-shared_Makefile_00':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => 'PREFIX=/usr/local',
    require => [ Archive['bzip2-shared'], ],
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_01':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => 'install: all'
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_02':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\ttest -d $(PREFIX) || mkdir -p $(PREFIX)",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_03':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\ttest -d $(PREFIX)/lib || mkdir -p $(PREFIX)/lib",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_04':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tinstall -m 0755 libbz2.so.1.0.6 $(PREFIX)/lib",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_05':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tln -s libbz2.so.1.0.6 libbz2.so.1.0",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_06':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tln -s libbz2.so.1.0 libbz2.so",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_07':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tinstall -m 0755 libbz2.so.1.0 $(PREFIX)/lib",
  } ~>
  file_line { 'correct_bzip2-shared_Makefile_08':
    path    => '/robotx/toolchain/src/bzip2-shared/Makefile-libbz2_so',
    line    => "\tinstall -m 0755 libbz2.so $(PREFIX)/lib",
  } ~>
  exec { 'correct_bzip2-shared_Makefile_09':
    command => "sed -i 's/\$(CC) -shared/\$(CC) \$(CFLAGS) -shared/' Makefile-libbz2_so",
    cwd     => '/robotx/toolchain/src/bzip2-shared',
    path    =>  [ '/robotx/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
  } ~>
  exec { 'correct_bzip2-shared_Makefile_10':
    command => "sed -i 's/^CFLAGS=/CFLAGS +=/' Makefile-libbz2_so",
    cwd     => '/robotx/toolchain/src/bzip2-shared',
    path    =>  [ '/robotx/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
  }

  # Download each archive and spawn Installers for each one.
  $archives.each |String $archive,
                  Struct[{'url' => String,
                          Optional['creates'] => String,
                          Optional['args'] => Hash,
                          Optional['require'] => Tuple[Any, 1, default],
                          'method' => String,
                          Optional['src_dir'] => String,
                          Optional['prebuild'] => String,
                          Optional['postbuild'] => String}] $params| {

        $extension = $params['url'] ? {
          /.*\.zip/       => 'zip',
          /.*\.tgz/       => 'tgz',
          /.*\.tar\.gz/   => 'tar.gz',
          /.*\.txz/       => 'txz',
          /.*\.tar\.xz/   => 'tar.xz',
          /.*\.tbz/       => 'tbz',
          /.*\.tbz2/      => 'tbz2',
          /.*\.tar\.bz2/  => 'tar.bz2',
          /.*\.h/         => 'h',
          /.*\.hpp/       => 'hpp',
          default         => 'UNKNOWN',
        }

        archive { "${archive}":
          url              => $params['url'],
          target           => "/robotx/toolchain/src/${archive}",
          src_target       => "/robotx/toolchain/src",
          purge_target     => true,
          checksum         => false,
          follow_redirects => true,
          timeout          => 0,
          extension        => $extension,
          strip_components => 1,
          root_dir         => '.',
          require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        }
        installer { "${archive}":
          archs       => $archs,
          creates     => $params['creates'],
          require     => delete_undef_values(flatten([ Archive["${archive}"], $params['require'], Class['installer::prerequisites'], Class['dev_tools'], ])),
          args        => $params['args'],
          src_dir     => $params['src_dir'],
          prebuild    => $params['prebuild'],
          postbuild   => $params['postbuild'],
          method      => $params['method'],
          extension   => $extension,
        }
  }

  # Install protobuf.
  class { 'protobuf': }

  # Install catch.
  installer { 'catch':
    url       => 'https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp',
    archs     => $archs,
    extension => 'hpp',
    method    => 'wget',
  }

  archive { "Spinnaker":
    url              => "http://nubots.net/tarballs/spinnaker_1_0_0_295_amd64.tar.gz",
    target           => "/robotx/toolchain/src/Spinnaker",
    src_target       => "/robotx/toolchain/src",
    purge_target     => true,
    checksum         => false,
    follow_redirects => true,
    timeout          => 0,
    extension        => "tar.gz",
    strip_components => 1,
    root_dir         => '.',
    require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
  }

  # Perform any complicated postbuild instructions here.
  $archs.each |String $arch, Hash $params| {
    exec { "${arch}_Spinnaker_Files":
      creates  => "/robotx/toolchain/$arch/include/Spinnaker.h",
      command  => "cd include && cp -r ./* /robotx/toolchain/$arch/include/ && cd .. &&
                   cd lib &&
                   cp libGCBase_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cp libGenApi_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cp libLog_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cp libMathParser_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cp libNodeMapData_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cp libptgreyvideoencoder.so* /robotx/toolchain/$arch/lib/ &&
                   cp libSpinnaker.so* /robotx/toolchain/$arch/lib/ &&
                   cp libXmlParser_gcc540_v3_0.so* /robotx/toolchain/$arch/lib/ &&
                   cd ..",
      cwd      => "/robotx/toolchain/src/Spinnaker",
      path     =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                     '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout  => 0,
      provider => 'shell',
      require  => Archive['Spinnaker'],
    }
  }

  $archs.each |String $arch, Hash $params| {
    # Create CMake toolchain files.
    $prefix          = '/robotx/toolchain'
    $compile_options = join(prefix(suffix($params['flags'], ')'), 'add_compile_options('), "\n")
    $compile_params  = join($params['params'], " ")

    file { "${arch}.cmake":
      content =>
"set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_FIND_ROOT_PATH \"${prefix}/${arch}\"
       \"${prefix}\"
       \"/usr/local\"
       \"/usr\")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

${compile_options}

include_directories(SYSTEM \"${prefix}/${arch}/include\")
include_directories(SYSTEM \"${prefix}/include\")

set(CMAKE_C_FLAGS \"\${CMAKE_C_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} ${compile_params}\" CACHE STRING \"\" FORCE)

set(PLATFORM \"${arch}\" CACHE STRING \"The platform to build for.\" FORCE)
",
      ensure  => present,
      path    => "${prefix}/${arch}.cmake",
    }
  }
}

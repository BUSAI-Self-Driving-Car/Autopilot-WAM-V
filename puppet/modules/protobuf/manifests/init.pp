# == Class: protobuf
class protobuf {
    # Build ELLCC
    archive { 'protobuf-native':
      url              => 'https://github.com/google/protobuf/releases/download/v3.1.0/protobuf-cpp-3.1.0.tar.gz',
      target           => '/robotx/toolchain/src/protobuf',
      src_target       => '/robotx/toolchain/src',
      purge_target     => true,
      checksum         => false,
      follow_redirects => true,
      timeout          => 0,
      extension        => 'tar.gz',
      strip_components => 1,
      root_dir         => '.',
      require          => [ Class['installer::prerequisites'], Class['dev_tools'], ],
    }

    exec { 'autotools_protobuf':
      creates     => '/robotx/toolchain/bin/protoc',
      command     => "cp protobuf-native.tar.gz protobuf.tar.gz &&
                      cd protobuf &&
                      ./configure --prefix=\"/robotx/toolchain\" --with-zlib &&
                      make -j\$(nproc) &&
                      make install",
      cwd         => '/robotx/toolchain/src',
      path        =>  [ '/robotx/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
      timeout     => 0,
      provider    => 'shell',
      require     => [ Archive['protobuf-native'], ],
    }
}


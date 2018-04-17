# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|

  # Settings for a parallels provider
  config.vm.provider "parallels" do |v, override|
    # Use parallels virtualbox
    override.vm.box = "parallels/ubuntu-16.04"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    # and http://parallels.github.io/vagrant-parallels/docs/configuration.html
    v.memory = 8192
    v.cpus = 4
    v.update_guest_tools = true
  end

  # Settings if using a virtualbox provider
  config.vm.provider "virtualbox" do |v, override|
    # Use the official ubuntu box
    #override.vm.box = "ubuntu/xenial64"

    # Use custom box because official Ubuntu one is shit.
    override.vm.box = "nubots_xenial64"
    override.vm.box_url = "https://uoneduau-my.sharepoint.com/personal/c3124185_uon_edu_au/_layouts/15/guestaccess.aspx?guestaccesstoken=%2bndUQAG0OSfCUAOTNpjewJE%2bv0slyCCyfeEonMpJwpU%3d&docid=03ac69c7a811041dba5afe5247c253a59&rev=1"

    # See http://www.virtualbox.org/manual/ch08.html#vboxmanage-modifyvm
    v.memory = 8192
    v.cpus = 4
    v.customize ["modifyvm", :id, "--vram", 128]
    v.customize ["modifyvm", :id, "--ioapic", "on"]
    v.customize ["modifyvm", :id, "--accelerate3d", "on"]
    v.customize ["modifyvm", :id, "--cableconnected1", "on"]
    v.customize ["modifyvm", :id, "--natdnshostresolver1", "on"]
  end

  # Fix the no tty error when installing
  config.vm.provision "fix-no-tty", type: "shell" do |shell|
    shell.privileged = false
    shell.inline = "sudo sed -i '/tty/!s/mesg n/tty -s \\&\\& mesg n/' /root/.profile"
  end

  # Before the puppet provisioner runs
  # install puppet modules that are used
  config.vm.provision "install-puppet-modules", type: "shell" do |shell|
    shell.inline = "apt-get install -y puppet;
                    mkdir -p /etc/puppet/modules;
                    puppet module list | grep -q 'puppetlabs-apt' \\
                         || puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0;
                    puppet module list | grep -q 'puppetlabs-vcsrepo' \
                         || puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'camptocamp-archive' \
                         || puppet module install camptocamp-archive --module_repository https://forge.puppet.com;
                    puppet module list | grep -q 'maestrodev-wget' \
                         || puppet module install maestrodev-wget --module_repository https://forge.puppet.com;"
  end

  # Enable provisioning with Puppet stand alone.  Puppet manifests
  # are contained in a directory path relative to this Vagrantfile.
  config.vm.provision :puppet do |puppet|
    puppet.manifests_path = "puppet/manifests"
    puppet.module_path = "puppet/modules"
    puppet.manifest_file = "robotx.pp"
    puppet.options = [
      # See https://docs.puppetlabs.com/references/3.6.2/man/agent.html#OPTIONS
      "--verbose",
      "--debug",
      "--parser=future"
    ]
  end

  # This VM will build all dependencies by source (use this to update old dependencies, or to generate a new deb file)
  config.vm.define "robotx", primary: false do |machine|
    machine.vm.hostname = "robotx.newcastle.edu.au"
    machine.vm.synced_folder ".", "/home/vagrant/Autopilot"
  end
end

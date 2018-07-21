#！/bin/bash
#mkdir /usr/share/cppad_ipopt
apt-get install cppad
apt-get install gfortran
apt-get install unzip
unzip Ipopt-3.12.7.zip -d /usr/share/cppad_ipopt
#Blas
cp blas-20130815.tgz /usr/share/cppad_ipopt/Ipopt-3.12.7/ThirdParty/Blas
#Lapack
cp lapack-3.4.2.tgz /usr/share/cppad_ipopt/Ipopt-3.12.7/ThirdParty/Lapack
#ASL
cp 1.3.0.tar.gz /usr/share/cppad_ipopt/Ipopt-3.12.7/ThirdParty/ASL
#MUMPS
cp MUMPS_4.10.0.tar.gz /usr/share/cppad_ipopt/Ipopt-3.12.7/ThirdParty/Mumps
cp install_ipopt.sh /usr/share/cppad_ipopt
cd /usr/share/cppad_ipopt
bash install_ipopt.sh Ipopt-3.12.7/
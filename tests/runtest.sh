set -e
mylib=`find .. -name libPRISM-omp-wrapper.so`
if [ ! -f $mylib ]; then
    echo "Unable to find wrapper library"
    exit -1
fi

set +e
echo Executing:
echo LD_PRELOAD=$mylib ./base
LD_PRELOAD=$mylib ./base
result=$?
set -e

if [ $result -ne 0 ]; then
    echo "Test FAILED"
    exit -1
else
    echo "Test ok"
    exit 0
fi

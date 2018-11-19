raspivid  -t 0 -w 640 -h 480 -hf -ih -fps 30 --rotation 180 -o - | nc -k -l 5557

LOGIN="cs184-bu"
echo "Which s349 machine? (1-14)"
serv_num=`read`

remote="$LOGIN"@s349-"$serv_num"@cs.berkeley.edu

ssh $remote touch t

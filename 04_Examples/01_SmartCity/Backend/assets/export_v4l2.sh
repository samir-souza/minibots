echo v4l2-ctl -c $(v4l2-ctl --all -k | grep -E '\(int)|\(menu)|\(bool)' | sed -E 's/^ *//;s/^([^ ]*).*value=([^ ]*).*/\1=\2/' | tr '\n' , | head -c -1)

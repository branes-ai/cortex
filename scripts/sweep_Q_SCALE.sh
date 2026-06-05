export CORTEX_EUROC_MH05=/srv/samba/sw-21/EuRoC-MAV-dataset/machine_hall/MH_05_difficult/mav0
export CORTEX_EUROC_V203=/srv/samba/sw-21/EuRoC-MAV-dataset/vicon_room2/V2_03_difficult/mav0
V2="V2_03_difficult replay (moving start) converges"
MH="MH_05_difficult replay (moving start) bootstraps and stays bounded"
for k in 1 3 10 30 100; do
  for name in "$V2" "$MH"; do
    CORTEX_Q_SCALE=$k ./build/sitl-release/tests/vio_euroc "$name" 2>&1 \
      | grep -E 'init=static.*ATE|NEES over|per-block NEES|position=' \
      | tr '\n' ' ' | sed "s/^/[Q=$k] /; s/\$/\n/"
  done
done
echo "===== DONE =====")

echo "Run:"
#echo "mkdir -p $(find map_server)/map/ && cd $(find map_server)/map/"
echo "source devel/setup.bash"
echo "cd /map"
echo "rosrun map_server map_saver"
#docker compose --profile playback --profile visualizaiton --profile filter --profile slamp exec bag_player bash
#source devel/setup.bash
docker compose exec slam bash
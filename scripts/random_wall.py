import argparse
import random
import rospy

from std_srvs.srv import Trigger

def random_wall(num_segments, num_layers, suffix, orange_bricks=False):
    sizes = {'B': 4, 'G': 2, 'R': 1}

    out_file = '../params/wall_{}.txt'.format(suffix)

    if num_segments == 0 or num_layers == 0:
        with open('../params/wall_default.txt', 'r') as default, open(out_file, 'w') as output:
            output.write(default.read())
            return

    layer_strings = []
    o_segment = num_segments - 1 if orange_bricks else -1
    for layer in range(num_layers):
        layer_list = []
        for segment in range(num_segments):
            amount = {'B': 1, 'G': 2, 'R': 4}
            if segment == o_segment:
                bricks_layer = ['O', 'O']
            else:
                bricks_layer = []
                while True:
                    m = random.random() * sum(amount.values())
                    if m < amount['R']:
                        bricks_layer.append('R')
                        amount['R'] -= 1
                    elif m < amount['R'] + amount['G']:
                        bricks_layer.append('G')
                        amount['G'] -= 1
                    else:
                        bricks_layer.append('B')
                        amount['B'] -= 1

                    if sum([sizes[brick] for brick in bricks_layer]) >= 12:
                        break
            layer_list.extend(bricks_layer)

        layer_strings.append(' '.join(layer_list))

    string = '\n'.join(layer_strings)

    with open(out_file, 'w') as stream:
        stream.write(string)

    try:
        rospy.wait_for_service('/restart', timeout=1)
        restart = rospy.ServiceProxy('/restart', Trigger)
        restart()
    except rospy.ROSException:
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("num_segments", help="Number of wall segments", type=int)
    parser.add_argument("num_layers", help="Number of layers in each segment", type=int)
    parser.add_argument("suffix", help="UAV or UGV", type=str)
    parser.add_argument("--orange", help="Use orange bricks in the random wall", action="store_true")
    args = parser.parse_args()

    random_wall(args.num_segments, args.num_layers, args.suffix, orange_bricks=args.orange)

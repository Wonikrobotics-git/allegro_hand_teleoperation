from geort.replay_mocap import ReplayMocap
from geort.env.hand import HandKinematicModel
from geort import load_model, get_config
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--hand', type=str, default='allegro')
    parser.add_argument('--ckpt', type=str)
    parser.add_argument('--data', type=str, default='human')
    parser.add_argument('--use_last', action='store_true',
                        help='Load last checkpoint instead of best checkpoint (default: best)')

    args = parser.parse_args()

    # GeoRT Model.
    epoch_to_load = 0 if args.use_last else 'best'
    model = load_model(args.ckpt, epoch=epoch_to_load)


    # Motion Capture.
    mocap = ReplayMocap(args.data)

    # Robot Simulation.
    config = get_config(args.hand)
    hand = HandKinematicModel.build_from_config(config, render=True)
    viewer_env = hand.get_viewer_env()

    # Run!
    while True:
        for i in range(10):
            viewer_env.update()

        result = mocap.get()

        if result['status'] == 'recording' and result["result"] is not None:
            points = result["result"]
            qpos = model.forward(points)

            hand.set_qpos_target(qpos)

        if result['status'] == 'quit':
            break

if __name__ == '__main__':
    main()

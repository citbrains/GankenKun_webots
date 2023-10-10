from marllib import marl
#import soccer_v0

from rllib import RLlibMPE
ENV_REGISTRY = {}
ENV_REGISTRY["mpe"] = RLlibMPE

env = marl.make_env(environment_name="mpe", map_name="simple_spread", force_coop=True)
mappo = marl.algos.mappo(hyperparam_source="mpe")
model = marl.build_model(env, mappo, {"core_arch": "mlp", "encode_layer": "128-256"})
mappo.fit(env, model, stop={'episode_reward_mean': 2000, 'timesteps_total': 20000000}, local_mode=False, num_gpus=1,
          num_workers=10, share_policy='all', checkpoint_freq=500)

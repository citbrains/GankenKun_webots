import rllib
from marllib import marl

ENV_REGISTRY = {}
ENV_REGISTRY["soccer"] = rllib.RLlibSoccer

env = marl.make_env(environment_name="soccer", map_name="soccer")
mappo = marl.algos.mappo(hyperparam_source="test")
model = marl.build_model(env, mappo, {"core_arch": "mlp", "encode_layer": "128-256"})
mappo.fit(env, model, stop={'episode_reward_mean': 2000, 'timesteps_total': 10000000}, local_mode=True, num_gpus=1,
          num_workers=1, share_policy='all', checkpoint_freq=50)

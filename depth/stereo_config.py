import copy
import json
import pprint
import optuna


class StereoConfig:
    '''Class for managing dai.node.StereoDepth node configuration.'''

    def __init__(self, cfg={}):
        self._cfg = cfg

    @classmethod
    def load(cls, filename):
        '''Load dai.node.StereoDepth node configuration from a file.'''
        with open(filename, 'r') as f:
            cfg = json.load(f)
            return cls(cfg)

    @classmethod
    def from_optuna(cls, storage=None, study=None, study_name=None, trial_id=0):
        '''Create StereoConfig from an optuna study.'''
        import optuna
        if study is None:
            study = optuna.load_study(study_name=study_name, storage=storage)
        elif storage is None:
            raise ValueError('Either `storage` or `study` must be given')

        cfg = study.trials[trial_id].params
        return cls(cfg)

    @classmethod
    def sample_from_json(cls, params, trial: optuna.Trial):
        """
        Create a config by sampling using from optuna.Trial
        params is an iterable with the following content:
        (key: str, (type: str, *args[, **kwargs]) [, (prerequisite_key, prerequisite_value)])
        TODO: add support for permutations over an list of values
        """
        cfg = {}

        def suggest(cfg, trial, key, type, *args, **kwargs):
            cfg[key] = getattr(trial, 'suggest_' + type)(key, *args, **kwargs)

        for p in params:
            key, args, *condition = p
            if condition:
                ckey, cvalue = condition[0]
                if cfg.get(ckey, cvalue) != cvalue:
                    continue
            try:
                try:
                    # Try with the last argument being a dictionary
                    suggest(cfg, trial, key, *args[:-1], **args[-1])
                except TypeError:
                    suggest(cfg, trial, key, *args)
            except ValueError:
                # value not in study parameters
                continue

        return cls(cfg)

    @classmethod
    def from_jsonfile(cls, json_filename, trial: optuna.Trial):
        """Create StereoConfig using configuration from a json file using optuna.Trial to sample from the parameter space."""
        params = json.load(json_filename)
        return cls.sample_from_json(params, trial)

    def __delitem__(self, key):
        del self._cfg[key]

    def update(self, a_cfg: 'StereoConfig'):
        '''Update configuration with another instance of StereoConfig, same as dictionary.update().'''
        self._cfg.update(a_cfg._cfg)

    def copy(self):
        '''Return deep copy of the StereoConfig.'''
        return StereoConfig(copy.deepcopy(self._cfg))

    def save(self, filename):
        '''Save dai.node.StereoDepth node configuration to a file.'''
        with open(filename, 'w') as f:
            json.dump(self._cfg, f, indent=4)

    def __str__(self):
        return 'StereoConfig:' + pprint.pformat(self._cfg)

    def configure_stereo_node(self, stereo):
        '''Apply configuration to a dai.node.StereoDepth node.'''
        config = copy.deepcopy(self._cfg)
        import depthai as dai  # This is needed for the eval and exec calls below.

        # Typos in previous version of optimize_depth_params
        try:
            config['stereo.initialConfig.costAggregation.p1Config.enableAdaptive'] = config['stereo.initialConfig.costAggregation.p1Config,enableAdaptive']
            del (config['stereo.initialConfig.costAggregation.p1Config,enableAdaptive'])
        except KeyError:
            pass
        try:
            config['stereo.initialConfig.costAggregation.p2Config.edgeValue'] = config['stereo.initialConfig.costAggregation.p2Config.edegeValue']
            del (config['stereo.initialConfig.costAggregation.p2Config.edegeValue'])
        except KeyError:
            pass

        try:
            # Split filtering cfg.postProcessing.filteringOrder if composed of a single string
            config['cfg.postProcessing.filteringOrder'] = config['cfg.postProcessing.filteringOrder'].split(',')
        except (KeyError, AttributeError):
            pass

        try:
            # Apply the preset first, so that it can be overriden by the rest of the config
            eval(f"stereo.setDefaultProfilePreset({config['stereo.setDefaultProfilePreset']})")
            del config['stereo.setDefaultProfilePreset']
            # print('INFO: Applying preset and possibly overriding it with other specific settings.')
        except KeyError:
            pass

        cfg = None
        unused_keys = list(config.keys())
        # We need to set the stereo params first, then initial config (cfg)
        for key_prefix in ('stereo.', 'cfg.'):
            for k, v in config.items():
                if not k.startswith(key_prefix):
                    continue
                # print(f'{k} -> {v}')
                k0, _, kN = k.partition('.')
                if k0 == 'cfg' and cfg is None:
                    # Get initialconfig on first request
                    cfg = stereo.initialConfig.get()
                    # workaround for versions 2.28.0.0 and lower
                    if not hasattr(cfg.postProcessing, 'filteringOrder') and 'cfg.postProcessing.filteringOrder' in config:
                        # TODO This might cause a KeyError, but only on 2.28.0.0..
                        print('WARNING: Ignoring cfg.postProcessing.filteringOrder settings, since it is not implemented in this version of depthai')
                        unused_keys.remove('cfg.postProcessing.filteringOrder')
                        del config['cfg.postProcessing.filteringOrder']

                    # print(' - stereo.initialConfig.get()')
                if isinstance(v, (list, tuple)):
                    v = ','.join(v)
                try:
                    # print(f' - trying calling {k} as a function')
                    try:
                        eval(f'{k}({v})')
                        unused_keys.remove(k)
                    except NameError:
                        eval(f'{k}(dai.StereoDepthConfig.{v})')  # workaround old naming
                        unused_keys.remove(k)
                    # fn(v)
                except (TypeError, SyntaxError):  # not callable, try setting it directly
                    # print(f' - trying setting {k} as a variable')
                    # rsetattr(locals()[k0], kN, v)
                    # print(f' - {k}={v}')
                    try:
                        exec(f'{k}={v}')
                        unused_keys.remove(k)
                    except NameError:
                        exec(f'{k}=dai.StereoDepthConfig.{v}')  # workaround old naming
                        unused_keys.remove(k)
                    # print('  => ', rgetattr(locals()[k0], kN))
        if not cfg is None:
            stereo.initialConfig.set(cfg)

        if len(unused_keys):
            raise ValueError('Unused configuration keys:', unused_keys)

    def __setitem__(self, key, value):
        self._cfg[key] = value

    def __getitem__(self, key):
        return self._cfg[key]

    def get(self, key, value=None):
        return self._cfg.get(key, value)

    def __contains__(self, key):
        return key in self._cfg

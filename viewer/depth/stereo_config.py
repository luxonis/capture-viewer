import copy
import json
import pprint
import optuna


class StereoConfig:
    '''Class for managing dai.node.StereoDepth node configuration.'''

    def __init__(self, cfg={}):
        self._cfg = {}
        for k, v in cfg.items():
            self[k] = v

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

    @classmethod
    def from_stereo_node_cfg(cls, _cfg):
        """Get configuration from a dai.node.StereoDepth node config."""
        return StereoConfig({
            'stereo.setRectification': True,  # TODO use enableRectification
            'cfg.costMatching.confidenceThreshold': _cfg.costMatching.confidenceThreshold,
            # algorithmControl
            'stereo.setDepthAlign': 'dai.StereoDepthConfig.AlgorithmControl.DepthAlign.' + _cfg.algorithmControl.depthAlign.name,
            'cfg.algorithmControl.depthUnit': 'dai.StereoDepthConfig.AlgorithmControl.DepthUnit.' + _cfg.algorithmControl.depthUnit.name,
            'cfg.algorithmControl.customDepthUnitMultiplier': _cfg.algorithmControl.customDepthUnitMultiplier,
            'cfg.algorithmControl.enableLeftRightCheck': _cfg.algorithmControl.enableLeftRightCheck,
            'cfg.algorithmControl.enableExtended': _cfg.algorithmControl.enableExtended,
            'cfg.algorithmControl.enableSubpixel': _cfg.algorithmControl.enableSubpixel,
            'cfg.algorithmControl.leftRightCheckThreshold': _cfg.algorithmControl.leftRightCheckThreshold,
            'cfg.algorithmControl.subpixelFractionalBits': _cfg.algorithmControl.subpixelFractionalBits,
            'cfg.algorithmControl.disparityShift': _cfg.algorithmControl.disparityShift,
            # 'cfg.algorithmControl.centerAlignmentShiftFactor': _cfg.algorithmControl.centerAlignmentShiftFactor,
            'cfg.algorithmControl.numInvalidateEdgePixels': _cfg.algorithmControl.numInvalidateEdgePixels,
            # postProcessing
            'cfg.postProcessing.filteringOrder': tuple('dai.StereoDepthConfig.PostProcessing.' + str(f) for f in _cfg.postProcessing.filteringOrder),
            'cfg.postProcessing.median': 'dai.StereoDepthConfig.MedianFilter.' + _cfg.postProcessing.median.name,
            'cfg.postProcessing.bilateralSigmaValue': _cfg.postProcessing.bilateralSigmaValue,
            'cfg.postProcessing.spatialFilter.enable': _cfg.postProcessing.spatialFilter.enable,
            'cfg.postProcessing.spatialFilter.holeFillingRadius': _cfg.postProcessing.spatialFilter.holeFillingRadius,
            'cfg.postProcessing.spatialFilter.alpha': _cfg.postProcessing.spatialFilter.alpha,
            'cfg.postProcessing.spatialFilter.delta': _cfg.postProcessing.spatialFilter.delta,
            'cfg.postProcessing.spatialFilter.numIterations': _cfg.postProcessing.spatialFilter.numIterations,
            'cfg.postProcessing.temporalFilter.enable': _cfg.postProcessing.temporalFilter.enable,
            'cfg.postProcessing.temporalFilter.persistencyMode': 'dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.' + _cfg.postProcessing.temporalFilter.persistencyMode.name,
            'cfg.postProcessing.temporalFilter.alpha': _cfg.postProcessing.temporalFilter.alpha,
            'cfg.postProcessing.temporalFilter.delta': _cfg.postProcessing.temporalFilter.delta,
            'cfg.postProcessing.thresholdFilter.minRange': _cfg.postProcessing.thresholdFilter.minRange,
            'cfg.postProcessing.thresholdFilter.maxRange': _cfg.postProcessing.thresholdFilter.maxRange,
            'cfg.postProcessing.brightnessFilter.minBrightness': _cfg.postProcessing.brightnessFilter.minBrightness,
            'cfg.postProcessing.brightnessFilter.maxBrightness': _cfg.postProcessing.brightnessFilter.maxBrightness,
            'cfg.postProcessing.speckleFilter.enable': _cfg.postProcessing.speckleFilter.enable,
            'cfg.postProcessing.speckleFilter.speckleRange': _cfg.postProcessing.speckleFilter.speckleRange,
            'cfg.postProcessing.speckleFilter.differenceThreshold': _cfg.postProcessing.speckleFilter.differenceThreshold,
            'cfg.postProcessing.decimationFilter.decimationFactor': _cfg.postProcessing.decimationFilter.decimationFactor,
            'cfg.postProcessing.decimationFilter.decimationMode': 'dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.' + _cfg.postProcessing.decimationFilter.decimationMode.name,
            # censusTransform
            'cfg.censusTransform.enableMeanMode': _cfg.censusTransform.enableMeanMode,
            'cfg.censusTransform.kernelSize': 'dai.StereoDepthConfig.CensusTransform.KernelSize.' + _cfg.censusTransform.kernelSize.name,
            'cfg.censusTransform.kernelMask': _cfg.censusTransform.kernelMask,
            'cfg.censusTransform.threshold': _cfg.censusTransform.threshold,
            # costMatching
            'cfg.costMatching.disparityWidth': 'dai.StereoDepthConfig.CostMatching.DisparityWidth.' + _cfg.costMatching.disparityWidth.name,
            'cfg.costMatching.enableCompanding': _cfg.costMatching.enableCompanding,
            'cfg.costMatching.invalidDisparityValue': _cfg.costMatching.invalidDisparityValue,
            # 'cfg.costMatching.confidenceThreshold': _cfg.costMatching.confidenceThreshold,
            'cfg.costMatching.linearEquationParameters.alpha': _cfg.costMatching.linearEquationParameters.alpha,
            'cfg.costMatching.linearEquationParameters.beta': _cfg.costMatching.linearEquationParameters.beta,
            'cfg.costMatching.linearEquationParameters.threshold': _cfg.costMatching.linearEquationParameters.threshold,
            # costAggregation
            'cfg.costAggregation.divisionFactor': _cfg.costAggregation.divisionFactor,
            'cfg.costAggregation.horizontalPenaltyCostP1': _cfg.costAggregation.horizontalPenaltyCostP1,
            'cfg.costAggregation.horizontalPenaltyCostP2': _cfg.costAggregation.horizontalPenaltyCostP2,
            'cfg.costAggregation.verticalPenaltyCostP1': _cfg.costAggregation.verticalPenaltyCostP1,
            'cfg.costAggregation.verticalPenaltyCostP2': _cfg.costAggregation.verticalPenaltyCostP2,
        })

    def __delitem__(self, key):
        del self._cfg[key]

    def update(self, a_cfg: 'StereoConfig'):
        '''Update configuration with another instance of StereoConfig, same as dictionary.update().'''
        for k, v in a_cfg._cfg.items():
            self[k] = v

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
        # print(f'DEBUG: StereoConfig.configure_stereo_node():\n  stereo_config: {self}')
        config = copy.deepcopy(self._cfg)
        import depthai as dai
        __is_dai3 = dai.__version__.startswith('3.')
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

        if 'stereo.setDefaultProfilePreset' in config:
            # Apply the preset first, so that it can be overriden by the rest of the config
            eval(f"stereo.setDefaultProfilePreset({config['stereo.setDefaultProfilePreset']})")
            del config['stereo.setDefaultProfilePreset']
            # print('INFO: Applying preset and possibly overriding it with other specific settings.')

        if __is_dai3:
            cfg = dai.StereoDepthConfig()
            cfg.algorithmControl = stereo.initialConfig.algorithmControl
            cfg.postProcessing = stereo.initialConfig.postProcessing
            cfg.costAggregation = stereo.initialConfig.costAggregation
            cfg.costMatching = stereo.initialConfig.costMatching
            cfg.censusTransform = stereo.initialConfig.censusTransform
        else:
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
                    if dai.__version__.startswith('2.'):
                        cfg = stereo.initialConfig.get()
                    else:
                        cfg = stereo.initialConfig
                    # workaround for versions 2.28.0.0 and lower
                    # if not hasattr(cfg.postProcessing, 'filteringOrder') and 'cfg.postProcessing.filteringOrder' in config:
                    #     # TODO This might cause a KeyError, but only on 2.28.0.0..
                    #     print('WARNING: Ignoring cfg.postProcessing.filteringOrder settings, since it is not implemented in this version of depthai')
                    #     unused_keys.remove('cfg.postProcessing.filteringOrder')
                    #     del config['cfg.postProcessing.filteringOrder']

                    # print(' - stereo.initialConfig.get()')
                if isinstance(v, (list, tuple)):
                    v = ','.join(v)
                try:
                    # print(f' - trying calling {k} as a function')
                    eval(f'{k}({v})')
                    unused_keys.remove(k)
                    # fn(v)
                except (TypeError, SyntaxError):  # not callable, try setting it directly
                    # print(f' - trying setting {k} as a variable')
                    # rsetattr(locals()[k0], kN, v)
                    # print(f' - {k}={v}')
                    exec(f'{k}={v}')
                    unused_keys.remove(k)
                    # print('  => ', rgetattr(locals()[k0], kN))
        if not cfg is None and dai.__version__.startswith('2.'):
            stereo.initialConfig.set(cfg)

        if len(unused_keys):
            raise ValueError('Unused configuration keys:', unused_keys)

    def configure_stereo_depth_config(self, cfg):
        '''Apply configuration to a dai.StereoDepthConfig.'''
        # print(f'DEBUG: StereoConfig.configure_stereo_depth_config():\n  stereo_config: {self}')
        config = copy.deepcopy(self._cfg)
        import depthai as dai
        __is_dai3 = dai.__version__.startswith('3.')
        assert not __is_dai3

        try:
            # Split filtering cfg.postProcessing.filteringOrder if composed of a single string
            config['cfg.postProcessing.filteringOrder'] = config['cfg.postProcessing.filteringOrder'].split(',')
        except (KeyError, AttributeError):
            pass

        unused_keys = list(config.keys())
        # We need to set the stereo params first, then initial config (cfg)
        for key_prefix in ('cfg.',):
            for k, v in config.items():
                if not k.startswith(key_prefix):
                    continue
                # print(f'{k} -> {v}')
                k0, _, kN = k.partition('.')
                # if k0 == 'cfg' and cfg is None:
                # workaround for versions 2.28.0.0 and lower
                # if not hasattr(cfg.postProcessing, 'filteringOrder') and 'cfg.postProcessing.filteringOrder' in config:
                #     # TODO This might cause a KeyError, but only on 2.28.0.0..
                #     print('WARNING: Ignoring cfg.postProcessing.filteringOrder settings, since it is not implemented in this version of depthai')
                #     unused_keys.remove('cfg.postProcessing.filteringOrder')
                #     del config['cfg.postProcessing.filteringOrder']

                # print(' - stereo.initialConfig.get()')
                if isinstance(v, (list, tuple)):
                    v = ','.join(v)
                try:
                    # print(f' - trying calling {k} as a function')
                    eval(f'{k}({v})')
                    unused_keys.remove(k)
                    # fn(v)
                except (TypeError, SyntaxError):  # not callable, try setting it directly
                    # print(f' - trying setting {k} as a variable')
                    # rsetattr(locals()[k0], kN, v)
                    # print(f' - {k}={v}')
                    exec(f'{k}={v}')
                    unused_keys.remove(k)
                    # print('  => ', rgetattr(locals()[k0], kN))

        # if len(unused_keys):
        #     raise ValueError('Unused configuration keys:', unused_keys)

    def __setitem__(self, key, value):
        _equivalent = (
            ('stereo.setSubpixel', 'cfg.algorithmControl.enableSubpixel'),
            ('stereo.setSubpixelFractionalBits', 'cfg.algorithmControl.subpixelFractionalBits'),
            ('stereo.setExtendedDisparity', 'cfg.algorithmControl.enableExtended'),
            ('stereo.setLeftRightCheck', 'cfg.algorithmControl.enableLeftRightCheck'),
            ('stereo.setConfidenceThreshold', 'cfg.costMatching.confidenceThreshold'),
            ('stereo.setMedianFilter', 'cfg.postProcessing.median'),
            ('stereo.initialConfig.setMedianFilter', 'cfg.postProcessing.median'),
        )

        for a, b in _equivalent:
            if key == a:
                # print(f'INFO: Using "{a}" is not recommended, use "{b}" instead, which should be quivalent')
                # raise NotImplementedError(f'Using {a} is not recommended, use {b} instead')
                if b in self._cfg:
                    raise ValueError(f'Config contains both {a} and {b}')
                key = b

        self._cfg[key] = value

    def __getitem__(self, key):
        return self._cfg[key]

    def get(self, key, value=None):
        return self._cfg.get(key, value)

    def __contains__(self, key):
        return key in self._cfg

    def keys(self):
        yield from self._cfg.keys()

    def items(self):
        yield from self._cfg.items()
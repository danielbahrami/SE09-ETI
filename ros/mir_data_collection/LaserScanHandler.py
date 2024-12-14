from sensor_msgs.msg import LaserScan

from . import sensor_data_handler

"""
Performed every 100ms. Does a 360 degree sweep with a lazer in 1 degree increments. contains a list with 360 items of distances in meters. Example data obtained from read_bag.py

Topic: /scan, Timestamp: 473987000000, Data: sensor_msgs.msg.LaserScan(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=473, nanosec=987000000), frame_id='virtual_laser_link'), angle_min=-3.1415927410125732, angle_max=3.1415927410125732, angle_increment=0.01745329238474369, time_increment=0.0, scan_time=0.03333333507180214, range_min=0.0, range_max=inf, ranges=[4.279360294342041, 4.25964879989624, 4.278565883636475, 4.278418064117432, 4.291310787200928, 4.2865777015686035, 4.311291217803955, 4.323256969451904, 4.315593719482422, 4.332608222961426, 4.350335597991943, 4.350996971130371, 4.38577938079834, 4.403662204742432, 4.417085647583008, 4.4327616691589355, 4.461036205291748, 4.462288856506348, 4.514033317565918, 4.535823822021484, 4.552627086639404, 4.597578525543213, 4.615677356719971, 4.650826930999756, 4.677966594696045, 4.7247161865234375, 4.7610955238342285, 4.818776607513428, 4.835258960723877, 4.901092052459717, 4.945131778717041, 4.993938446044922, 5.0463080406188965, 5.074866771697998, 5.188758850097656, 5.240807056427002, 5.310185432434082, 5.368495464324951, 5.424901962280273, 5.499655723571777, 5.607190132141113, 5.678004264831543, 5.763219833374023, 5.849688529968262, 5.953000545501709, 6.062673568725586, 6.145822048187256, 6.248601913452148, 6.433114528656006, 6.567943572998047, 6.697309970855713, 6.81373405456543, 12.263627052307129, 12.14540958404541, 11.965690612792969, 11.839866638183594, 11.710445404052734, 11.57614517211914, 11.458243370056152, 11.35520076751709, 11.168571472167969, 11.0794038772583, 10.965157508850098, 10.888227462768555, 10.782075881958008, 10.706574440002441, 10.634897232055664, 10.551945686340332, 10.485392570495605, 10.413778305053711, 10.358391761779785, 10.283489227294922, 10.191715240478516, 10.14122200012207, 10.081622123718262, 10.050422668457031, 10.012910842895508, 9.978181838989258, 9.951118469238281, 9.921730995178223, 9.888692855834961, 9.876158714294434, 9.839303016662598, 9.807907104492188, 9.79541015625, 9.782841682434082, 9.753999710083008, 9.752018928527832, 9.743322372436523, 9.750097274780273, 9.735818862915039, 9.746548652648926, 9.748795509338379, 9.756255149841309, 9.763084411621094, 9.774827003479004, 9.777925491333008, 9.808507919311523, 9.841804504394531, 9.862669944763184, 9.876203536987305, 9.910922050476074, 9.957304954528809, 9.981895446777344, 10.035587310791016, 10.084909439086914, 10.104748725891113, 10.182126998901367, 10.23728084564209, 10.29372501373291, 10.384187698364258, 10.425509452819824, 10.492916107177734, 10.571295738220215, 10.647476196289062, 10.730758666992188, 10.82811164855957, 10.923453330993652, 11.025904655456543, 11.122796058654785, 11.261263847351074, 11.355774879455566, 11.507247924804688, 11.633599281311035, 11.77412223815918, 11.89840030670166, 12.040300369262695, 12.161576271057129, 12.335485458374023, 12.50338363647461, 12.69368839263916, 12.893120765686035, 13.093360900878906, 13.271700859069824, 13.495224952697754, 13.787096977233887, 13.569281578063965, 13.414169311523438, 13.212092399597168, 12.999916076660156, 12.818120956420898, 12.634180068969727, 12.46346664428711, 12.309595108032227, 12.141470909118652, 12.003398895263672, 11.871249198913574, 11.716511726379395, 11.607735633850098, 11.469857215881348, 11.35147762298584, 11.243979454040527, 11.137700080871582, 11.051081657409668, 10.953904151916504, 10.85682487487793, 10.77701187133789, 10.696307182312012, 10.625143051147461, 10.553808212280273, 10.478534698486328, 10.425148010253906, 10.360209465026855, 10.269868850708008, 10.235745429992676, 10.184895515441895, 10.147509574890137, 10.111434936523438, 10.07900619506836, 10.035238265991211, 10.021440505981445, 9.987421035766602, 9.968281745910645, 9.933727264404297, 9.919394493103027, 9.91098403930664, 9.883759498596191, 9.878890037536621, 9.877379417419434, 9.872315406799316, 9.858894348144531, 9.874656677246094, 9.881485939025879, 9.891529083251953, 9.908666610717773, 9.93350601196289, 9.935606002807617, 9.949454307556152, 9.978466987609863, 10.018908500671387, 10.033117294311523, 10.069456100463867, 9.584651947021484, 8.887298583984375, 8.289559364318848, 7.776578903198242, 7.342151641845703, 6.255084037780762, 5.942556858062744, 5.66983699798584, 5.420104503631592, 5.190192699432373, 4.96917200088501, 4.796743869781494, 4.546812057495117, 4.408495903015137, 4.263041019439697, 4.1305365562438965, 3.9377806186676025, 3.842259168624878, 3.744231939315796, 3.652421236038208, 3.5403871536254883, 3.4448659420013428, 3.3290340900421143, 3.256441831588745, 3.2113144397735596, 3.109299898147583, 3.059403419494629, 2.9772300720214844, 2.9369561672210693, 2.8663666248321533, 2.812363386154175, 2.742310047149658, 2.687958002090454, 2.667991876602173, 2.5988032817840576, 2.5823633670806885, 2.5278241634368896, 2.5149710178375244, 2.453071117401123, 2.420654058456421, 2.400940179824829, 2.345097064971924, 2.331862449645996, 2.3026540279388428, 2.2776992321014404, 2.246485948562622, 2.225496530532837, 2.1992197036743164, 2.1921913623809814, 2.169402599334717, 2.1316301822662354, 2.1120493412017822, 2.089747190475464, 2.0899412631988525, 2.0516321659088135, 2.047952890396118, 2.035283088684082, 2.0149495601654053, 2.004286050796509, 2.0031816959381104, 1.9863216876983643, 1.9635342359542847, 1.9660075902938843, 1.9453353881835938, 1.936100721359253, 1.9465323686599731, 1.9258685111999512, 1.9206717014312744, 1.9253478050231934, 1.9021047353744507, 1.9137811660766602, 1.9006801843643188, 1.8969829082489014, 1.893662929534912, 1.892540454864502, 1.8953720331192017, 1.894140601158142, 1.900303840637207, 1.881285309791565, 1.9010461568832397, 1.90152108669281, 1.8927091360092163, 1.8955763578414917, 1.9055838584899902, 1.903207778930664, 1.9093385934829712, 1.9351181983947754, 1.912514090538025, 1.907131552696228, 1.9268254041671753, 1.9452792406082153, 1.9471379518508911, 1.9594489336013794, 1.957470417022705, 1.9729857444763184, 1.9765008687973022, 1.9919590950012207, 2.006289482116699, 2.0032148361206055, 2.036773681640625, 2.0443553924560547, 2.047074794769287, 2.070279121398926, 2.0903544425964355, 2.1094064712524414, 2.124166488647461, 2.145169496536255, 2.168957471847534, 2.1871936321258545, 2.2098324298858643, 2.2320289611816406, 2.2555882930755615, 2.275059938430786, 2.319084882736206, 2.350015878677368, 2.3762447834014893, 2.3848073482513428, 2.4342081546783447, 2.4634242057800293, 2.505774974822998, 2.544954299926758, 2.5790600776672363, 2.6400668621063232, 2.686495780944824, 2.7130050659179688, 2.7970263957977295, 2.8225317001342773, 2.9025278091430664, 2.9482386112213135, 2.999763250350952, 3.0727806091308594, 3.137768030166626, 3.217367172241211, 3.312788248062134, 3.3647923469543457, 3.4756131172180176, 3.542811870574951, 3.693772792816162, 3.7759194374084473, 3.914806604385376, 4.013372421264648, 4.177095890045166, 4.301156044006348, 4.51283073425293, 4.641750335693359, 4.850612163543701, 5.054321765899658, 5.278408050537109, 5.536346912384033, 5.759968280792236, 6.135904788970947, 6.400176048278809, 6.905570030212402, 7.2677788734436035, 7.793680191040039, 8.404711723327637, 8.938432693481445, 9.942093849182129, 9.949350357055664, 9.933969497680664, 9.9169340133667, 9.873950958251953, 9.851690292358398, 9.855731964111328, 9.850696563720703, 9.827422142028809, 9.803839683532715, 4.293968677520752], intensities=[]) 
"""


class LaserScanHandler(sensor_data_handler.SensorDataHandler):
    def __init__(self, database_upload_interval_seconds, bag_file_split_duration):
        super().__init__('/scan', LaserScan, database_upload_interval_seconds, bag_file_split_duration)

    def callback(self, msg):
        super().callback(msg)
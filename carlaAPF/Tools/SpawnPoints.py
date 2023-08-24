import enum


class Spawn_Points(enum.Enum):
    points = {'id_0': [-64.64484405517578, 24.471010208129883, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_1': [-67.25457000732422, 27.96375846862793, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_2': [-87.62303161621094, 12.967159271240234, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_3': [-84.9327621459961, 16.47465705871582, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_4': [-103.17900085449219, -14.434906959533691, 0.5999999642372131, 0.0, 0.0, -89.35775756835938],
              'id_5': [-106.64958953857422, -17.073978424072266, 0.5999999642372131, 0.0, 0.0, -89.35775756835938],
              'id_6': [-110.9637451171875, 59.68935775756836, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_7': [-114.43209075927734, 56.85029602050781, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_8': [-111.120361328125, 72.89886474609375, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_9': [-114.58869934082031, 70.05980682373047, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_10': [-24.33677864074707, -57.78562545776367, 0.5999999642372131, 0.0, 0.0, 0.5967350006103516],
              'id_11': [-110.19779968261719, -9.84222412109375, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_12': [-113.64817810058594, -14.281184196472168, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_13': [-109.92955780029297, -23.42840576171875, 0.5999950766563416, 0.0, 0.0, 90.6422348022461],
              'id_14': [-113.40350341796875, -25.76747703552246, 0.5999950766563416, 0.0, 0.0, 90.6422348022461],
              'id_15': [-56.86616134643555, 140.53555297851562, 0.5999999642372131, 0.0, 0.0, 0.35212698578834534],
              'id_16': [-54.34465789794922, 137.05099487304688, 0.5999999642372131, 0.0, 0.0, 0.35212698578834534],
              'id_17': [3.0477840900421143, 130.21006774902344, 0.5999999642372131, 0.0, 0.0, -179.6478271484375],
              'id_18': [5.62622594833374, 133.72601318359375, 0.5999999642372131, 0.0, 0.0, -179.6478271484375],
              'id_19': [45.76567459106445, 137.4599609375, 0.6000009179115295, 0.0, 0.0, 0.32044798135757446],
              'id_20': [48.546077728271484, 140.9755401611328, 0.6000009179115295, 0.0, 0.0, 0.32044798135757446],
              'id_21': [99.38441467285156, -6.305729389190674, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_22': [53.122718811035156, 133.94662475585938, 0.6000292897224426, 0.0, 0.0, -179.67953491210938],
              'id_23': [55.54227828979492, 130.46006774902344, 0.6000292897224426, 0.0, 0.0, -179.67953491210938],
              'id_24': [-52.07392120361328, 100.18904876708984, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_25': [-48.56741714477539, 102.47919464111328, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_26': [-52.07392120361328, 63.53809356689453, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_27': [-48.58211135864258, 60.62824249267578, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_28': [-41.66858673095703, 89.7454833984375, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_29': [-45.16095733642578, 92.45532989501953, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_30': [-15.148191452026367, 69.71400451660156, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_31': [-1.0131598711013794, 69.71400451660156, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_32': [98.8006591796875, 82.8908462524414, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_33': [14.130091667175293, 69.71400451660156, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_34': [6.006511211395264, 66.28325653076172, 0.5999999642372131, 0.0, 0.0, -179.92672729492188],
              'id_35': [-7.966956615447998, 66.28325653076172, 0.5999999642372131, 0.0, 0.0, -179.92672729492188],
              'id_36': [67.65974426269531, 69.8227767944336, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_37': [79.05529022216797, 69.8227767944336, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_38': [73.63292694091797, 66.35848999023438, 0.5999999642372131, 0.0, 0.0, -179.92672729492188],
              'id_39': [61.60259246826172, 66.35848999023438, 0.5999987721443176, 0.0, 0.0, -179.92672729492188],
              'id_40': [106.02881622314453, 67.41998291015625, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_41': [109.50276184082031, 71.24378967285156, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_42': [106.00283813476562, 92.81285095214844, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_43': [40.38964080810547, 41.94549560546875, 0.5999999642372131, 0.0, 0.0, 89.2488784790039],
              'id_44': [109.52326965332031, 89.83672332763672, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_45': [65.23527526855469, 13.414804458618164, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_46': [62.02554702758789, 16.90589141845703, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_47': [45.382850646972656, 13.414804458618164, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_48': [43.37312316894531, 16.90922737121582, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_49': [15.143111228942871, 16.681333541870117, 0.699999988079071, 0.0, 0.0, -179.84078979492188],
              'id_50': [20.45281219482422, 13.196090698242188, 0.699999988079071, 0.0, 0.0, -179.84078979492188],
              'id_51': [-20.11511993408203, 16.749099731445312, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_52': [-17.10540199279785, 13.25745677947998, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_53': [-0.7641560435295105, 24.61313247680664, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_54': [19.35021209716797, 137.4599609375, 0.5999999642372131, 0.0, 0.0, 0.32044798135757446],
              'id_55': [-3.973867654800415, 28.104215621948242, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_56': [19.600942611694336, 24.611188888549805, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_57': [17.091217041015625, 28.104215621948242, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_58': [77.00810241699219, 24.849666595458984, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_59': [74.79875183105469, 28.34353256225586, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_60': [10.912545204162598, -57.40138626098633, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_61': [7.4111151695251465, -60.89999771118164, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_62': [106.51315307617188, -21.554595947265625, 0.8999999761581421, 0.0, 0.0, -91.51957702636719],
              'id_63': [109.9569091796875, -27.333675384521484, 0.5999999642372131, 0.0, 0.0, -94.68233489990234],
              'id_64': [30.018199920654297, 133.94720458984375, 0.5999999642372131, 0.0, 0.0, -179.67953491210938],
              'id_65': [109.94696807861328, -17.187952041625977, 0.5999987721443176, 0.0, 0.0, -89.6092529296875],
              'id_66': [-6.554497718811035, 137.2259521484375, 0.5999999642372131, 0.0, 0.0, 0.35211899876594543],
              'id_67': [-41.85398864746094, -30.438610076904297, 0.6000707745552063, 0.0, 0.0, -89.56768035888672],
              'id_68': [-52.07392120361328, 82.65496826171875, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_69': [-28.581729888916016, 140.53555297851562, 0.5999999642372131, 0.0, 0.0, 0.35212698578834534],
              'id_70': [-18.385923385620117, 130.21006774902344, 0.5999999642372131, 0.0, 0.0, -179.6478271484375],
              'id_71': [-9.875875473022461, -57.55167007446289, 0.5999999642372131, 0.0, 0.0, 0.5967410802841187],
              'id_72': [11.176283836364746, -64.39876556396484, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_73': [-15.407496452331543, 133.7284698486328, 0.5999999642372131, 0.0, 0.0, -179.6478271484375],
              'id_74': [14.274852752685547, -67.89999389648438, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_75': [-110.76443481445312, 46.66007614135742, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_76': [99.07855987548828, 42.14179992675781, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_77': [-78.03414916992188, 12.967159271240234, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_78': [-71.26968383789062, 132.3148956298828, 0.5999999642372131, 0.0, 0.0, -167.12705993652344],
              'id_79': [-41.66887664794922, 48.905540466308594, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_80': [-87.27606201171875, 24.441530227661133, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_81': [-5.78348970413208, -64.5486831665039, 0.6000048518180847, 0.0, 0.0, -179.4032440185547],
              'id_82': [-27.800329208374023, -61.2840461730957, 0.5999999642372131, 0.0, 0.0, 0.5967350006103516],
              'id_83': [-27.16025161743164, 137.04421997070312, 0.5999999642372131, 0.0, 0.0, 0.35212698578834534],
              'id_84': [52.78997039794922, 69.8227767944336, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_85': [-89.88578796386719, 27.93427848815918, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_86': [-66.79419708251953, 12.99838924407959, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_87': [106.01924896240234, 50.86931228637695, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_88': [21.630592346191406, 140.97271728515625, 0.5999999642372131, 0.0, 0.0, 0.32044798135757446],
              'id_89': [-52.330810546875, -14.039613723754883, 0.5999999642372131, 0.0, 0.0, 90.43232727050781],
              'id_90': [-64.10392761230469, 16.505887985229492, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_91': [-76.66610717773438, 24.471010208129883, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_92': [99.43485260009766, -19.65771484375, 0.5999999642372131, 0.0, 0.0, 90.8448486328125],
              'id_93': [-25.51629638671875, 24.613134384155273, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_94': [-79.27583312988281, 27.96375846862793, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_95': [-45.34004592895508, -25.565021514892578, 0.6000707745552063, 0.0, 0.0, -89.56768035888672],
              'id_96': [-75.3438720703125, 16.47465705871582, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_97': [44.02894973754883, 52.54869842529297, 0.5999999642372131, 0.0, 0.0, -90.75109100341797],
              'id_98': [109.50276184082031, 53.29315185546875, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_99': [-52.310935974121094, -1.5852383375167847, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_100': [-106.68672943115234, -4.847458362579346, 0.5999999642372131, 0.0, 0.0, -89.35775756835938],
              'id_101': [-28.72602081298828, 28.104217529296875, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_102': [-48.81998825073242, -4.795074939727783, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_103': [-45.31125259399414, -1.6943949460983276, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_104': [-41.82510757446289, -6.604220867156982, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_105': [83.07522583007812, 13.414804458618164, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_106': [-68.73516845703125, 129.30384826660156, 0.5999999642372131, 0.0, 0.0, -167.12705993652344],
              'id_107': [59.81299591064453, 24.850223541259766, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_108': [-52.186737060546875, 42.56512451171875, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_109': [-67.0452880859375, -68.69316864013672, 0.5999999642372131, 0.0, 0.0, -179.4032440185547],
              'id_110': [-48.83995056152344, -17.213199615478516, 0.5999999642372131, 0.0, 0.0, 90.43232727050781],
              'id_111': [-45.317440032958984, -11.64532470703125, 0.6000024080276489, 0.0, 0.0, -90.16121673583984],
              'id_112': [102.56617736816406, 43.965667724609375, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_113': [-9.175959587097168, 140.70989990234375, 0.5999999642372131, 0.0, 0.0, 0.35211899876594543],
              'id_114': [29.235719680786133, 16.765228271484375, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_115': [-41.8338623046875, -16.555164337158203, 0.6000024080276489, 0.0, 0.0, -90.16121673583984],
              'id_116': [-48.67435073852539, 46.95527267456055, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_117': [80.26549530029297, 16.90700340270996, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_118': [27.14229393005371, 66.28325653076172, 0.5999999642372131, 0.0, 0.0, -179.92672729492188],
              'id_119': [57.40364456176758, 28.34353256225586, 0.5999999642372131, 0.0, 0.0, 0.15919798612594604],
              'id_120': [-103.21614074707031, -2.2083921432495117, 0.5999999642372131, 0.0, 0.0, -89.35775756835938],
              'id_121': [-64.58186340332031, -65.16736602783203, 0.5999999642372131, 0.0, 0.0, -179.4032440185547],
              'id_122': [-48.565467834472656, 85.6451187133789, 0.5999999642372131, 0.0, 0.0, 89.83876037597656],
              'id_123': [-13.339423179626465, -61.05009078979492, 0.5999999642372131, 0.0, 0.0, 0.5967410802841187],
              'id_124': [-27.022132873535156, 69.71400451660156, 0.5999999642372131, 0.0, 0.0, 0.07327299565076828],
              'id_125': [-114.23277282714844, 43.821014404296875, 0.5999999642372131, 0.0, 0.0, 90.6422348022461],
              'id_126': [-52.13356018066406, -40.1802978515625, 0.5999999642372131, 0.0, 0.0, 90.43230438232422],
              'id_127': [102.98018646240234, -22.705795288085938, 0.5999999642372131, 0.0, 0.0, 90.8448486328125],
              'id_128': [32.33772277832031, 130.46006774902344, 0.5999999642372131, 0.0, 0.0, -179.67953491210938],
              'id_129': [-41.491477966308594, 111.9452896118164, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_130': [-44.98299789428711, 114.95514678955078, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_131': [109.91348266601562, -6.925446510314941, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_132': [106.3773422241211, -1.6494430303573608, 0.5999999642372131, 0.0, 0.0, -89.6092529296875],
              'id_133': [102.31767272949219, 80.41472625732422, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_134': [32.04544448852539, 13.273029327392578, 0.5999999642372131, 0.0, 0.0, -179.84078979492188],
              'id_135': [-48.6427001953125, -43.35388946533203, 0.5999999642372131, 0.0, 0.0, 90.43230438232422],
              'id_136': [85.98224639892578, 66.35848999023438, 0.5999999642372131, 0.0, 0.0, -179.92672729492188],
              'id_137': [-45.149696350097656, 55.715389251708984, 0.5999999642372131, 0.0, 0.0, -90.16121673583984],
              'id_138': [-52.330810546875, -28.861330032348633, 0.5999999642372131, 0.0, 0.0, 90.43232727050781],
              'id_139': [-48.83995056152344, -32.03491973876953, 0.5999999642372131, 0.0, 0.0, 90.43232727050781],
              'id_140': [-41.7498779296875, -41.37368392944336, 0.5999999642372131, 0.0, 0.0, -89.56768035888672],
              'id_141': [-45.23593521118164, -36.50009536743164, 0.5999999642372131, 0.0, 0.0, -89.56768035888672],
              'id_142': [102.93003845214844, -9.381519317626953, 0.5999999642372131, 0.0, 0.0, 90.39070892333984],
              'id_143': [26.382587432861328, -57.40138626098633, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_144': [22.88115692138672, -60.89999771118164, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_145': [-15.448323249816895, -68.00706481933594, 0.6000048518180847, 0.0, 0.0, -179.4032440185547],
              'id_146': [-18.782678604125977, -64.70809173583984, 0.9000048637390137, 0.0, 0.0, -179.4032440185547],
              'id_147': [29.89404296875, -64.39876556396484, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_148': [32.992610931396484, -67.89999389648438, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_149': [47.55704879760742, -57.2252197265625, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_150': [44.055625915527344, -60.72383117675781, 0.5999999642372131, 0.0, 0.0, -0.0234375],
              'id_151': [54.46977233886719, -64.3486328125, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_152': [57.56834030151367, -67.849853515625, 0.5999999642372131, 0.0, 0.0, 179.9765625],
              'id_153': [-2.647038221359253, -68.04972076416016, 0.6000048518180847, 0.0, 0.0, -179.4032440185547],
              'id_154': [106.41629028320312, -12.711931228637695, 0.5999987721443176, 0.0, 0.0, -89.6092529296875], }


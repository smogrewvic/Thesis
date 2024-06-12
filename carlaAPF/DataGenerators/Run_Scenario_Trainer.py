import DataGenerators.AutopilotTrainer
import DataGenerators.EgoVehiclePygameTrainer
import TrafficGenerators.Archived.GenerateTraffic_Behavior
import multiprocessing


if __name__ == "__main__":


    p1 = multiprocessing.Process(target=DataGenerators.EgoVehiclePygameTrainer.main, args = ("id_98",)) #98, 113

    p2 = multiprocessing.Process(target=TrafficGenerators.Archived.GenerateTraffic_Behavior.main, args = (True,  # autopilot state
                                                                                                          30,  #percent speed limit
                                                                                                          0.01)) #sim_timestep


    p3 = multiprocessing.Process(target=DataGenerators.AutopilotTrainer.main, args=(['id_113'],  # destination
                                                                                 True,  # autopilot_on
                                                                                 False,  # display apf
                                                                                 False,  # display control system and actor positions
                                                                                 'none'))  # svo estimation type ('none', 'generic' 'type_1', 'type_2'

    p1.start()
    p2.start()
    p3.start()


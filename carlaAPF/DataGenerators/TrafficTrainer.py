import DataGenerators.AutopilotTrainer
import DataGenerators.EgoVehiclePygameTrainer
import TrafficGenerators.Archived.GenerateTraffic_Behavior
import multiprocessing


if __name__ == "__main__":


    p1 = multiprocessing.Process(target=DataGenerators.EgoVehiclePygameTrainer.main, args = ("id_98",)) #98, 113

    p2 = multiprocessing.Process(target=TrafficGenerators.Archived.GenerateTraffic_Behavior.main, args = (True,  # autopilot state
                                                                                                          30,  #percent speed limit
                                                                                                          0.01)) #sim_timestep

    p3 = multiprocessing.Process(target=DataGenerators.AutopilotTrainer.main, args=(True,  # autopilot_on
                                                                                    False,  # holonomic
                                                                                    True,  # display apf
                                                                                    False,  # display actors
                                                                                    False,  # display control system
                                                                                    False  # ego position
                                                                                    ))

    p1.start()
    p2.start()
    p3.start()


import json

class Pedestrian_Attributes:
    def __init__(self):
        pass
    @staticmethod
    def encode_string(data):
        """
        Encode the given data as a JSON string.

        Args:
            data (dict): A dictionary containing the behavior data.

        Returns:
            str: JSON-encoded string.
        """
        return json.dumps(data)

    @staticmethod
    def decode(json_str):
        """
        Decode a JSON string into a dictionary.

        Args:
            json_str (str): JSON-encoded string.

        Returns:
            dict: Decoded dictionary containing the behavior data.
        """
        return json.loads(json_str)

# Example usage:
data = {
    "behavior_type": "walking",
    "time_looking": 10,
    "time_waiting": 5,
    "distance_to_crosswalk": 20,
    "currently_crossing": True
}

# Encode the data as a JSON string
encoded_data = BehaviorDataEncoderDecoder.encode_string(data)
print("Encoded JSON:", encoded_data)

# Decode the JSON string back into a dictionary
decoded_data = BehaviorDataEncoderDecoder.decode(encoded_data)
print("Decoded Data:", decoded_data)
print(decoded_data['behavior_type'])
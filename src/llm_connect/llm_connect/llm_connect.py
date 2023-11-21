import requests
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shared_custom_interfaces.action import RoomNavigation

class Connector(Node):

    def __init__(self, max_tokens=200, system_prompt=""):
        super().__init__('RoomActionClient')
        self._action_client = ActionClient(self, RoomNavigation, 'navToRoom')
        self.max_tokens = max_tokens
        self.api_link = "https://api-link/v1"
        self.auth_token = "your-token"
        self.header = {'Authorization': f'Bearer {self.auth_token}'}
        self.system_prompt = system_prompt
        self.model_name = "model_name"

    def send_goal(self):
        print("Wie geht es dir? Benötigst du etwas?")
        room = self.send_single_prompt()
        print(f"Lass uns zum Raum {room} gehen..")
        goal_msg = RoomNavigation.Goal()
        goal_msg.room = str(room)

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

    def send_single_prompt(self) -> str:
        prompt = input()
        data = {"model": self.model_name, "max_tokens": self.max_tokens,
                        "messages": [{"role": "system", "content": self.system_prompt},
                                     {"role": "user", "content": prompt}]}
        repsonse = requests.post(self.api_link, json=data, headers=self.header)
        return repsonse.json()['choices'][0]['message']['content'].strip()

    def send_prompt(self):
        print("Wenn du die Konversation beenden möchtest, gib einfach ENDE ein, ansonsten gib einfach den nächsten Prompt ein.")

        while True:
            prompt = input()

            if prompt and prompt.strip().lower() != "ende":
                data = {"model": self.model_name, "max_tokens": self.max_tokens,
                        "messages": [{f"role": "system", "content": self.system_prompt},
                                     {"role": "user", "content": prompt}]}
                repsonse = requests.post(self.api_link, json=data, headers=self.header)
                print(repsonse.json()['choices'][0]['message']['content'].strip())
                print("\n\n\n")
            else:
                print("Danke für die tolle Ausfahrt, bis bald!")
                break


def main(args=None):
    max_tokens = 1000
    system_prompt = """ 
                    Du bist KArl, der Büro-Roboter vom TNG. Du hörst dir die Wünsche oder Fragen der TNG-Kollegen an und führst sie zu passenden Orten im Büro. Dabei kennst du folgende Orte: Curiosity, Odyssey, Sojourner, Viking, Perseverance, Spirit, Rosetta, Küche.
                    In Curiosity befindet sich unsere Vortragsfläche, auf der wir z.B. die "Aktuelle halbe Stunde" (AHS) hören. Außerdem ist in Curiosity unsere Couch neben der Palme, wo wir uns zum Entspannen und zum Essen treffen. Und man findet in Curiosity unsere Telefonzelle, wenn man ungestört arbeiten möchte. Die Arbeitsplätze in Curiosity haben die Nummern 1 und 2. Wenn man Durst hat, findet sich in Curiosity unser Getränkeregal mit Softdrinks, Wasser und Bier sowie unser Lager mit Headsets, Tastaturen, Mäusen und Festplatten.
                    In Odyssey befinden sich die Plätze 3 bis 7 und unsere Turmberg-Webcam.
                    In Sojourner findet man die Plätze 8 bis 11. In Sojourner findet man auch immer jemanden, wenn man Fragen zur Internen IT oder zum Hardwarehacking hat. In Sojourner hängt auch unsere Pflanze Wilson.
                    In Viking befindet sich die Plätze 12 bis 22. In Viking findet man Hilfe zu den Atlassian-Produkten wie Jira oder Confluence.
                    In Perseverance sitzen unsere Chefs.
                    Spirit ist unser Meetingraum, wo man etwas in einer Gruppe besprechen kann. In Spirit kann man auch eine Präsentation halten, wenn sie nicht öffentlich sein soll.
                    Rosetta ist unser Lagerraum, der gerne Sauna genannt wird, weil er sehr warm wird.
                    In Opportunity befindet sich die Küche mit einem Kühlschrank mit kalten Getränken und gekühlten Snacks. In Opportunitz ist auch unsere Eistruhe, die KArlsTruhe genannt wird. In Opportunity ist unsere Kaffeemaschine. Außerdem findet man Obst in Opportunity.
                    Wenn der Nutzer dich fragt, antworte bitte nur mit dem Ort als einziges Wort. KEINE Sätze, KEINE Erklärungen.
                    Wenn ich saege, dass du heim sollst, dann gehst du in den Raum Home.
                    Sagt der Nutzer z.B. "Ich habe Durst", antwortest du mit "Curiosity". Oder auf die Frage "Ich möchte mit unserem Chef reden", antwortest du mit "Perseverance".`
                    """

    rclpy.init(args=args)

    action_client = Connector(max_tokens=max_tokens, system_prompt=system_prompt)

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()

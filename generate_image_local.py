from html2image import Html2Image
import time

try:
    hti = Html2Image()
    # We load the local HTML which renders the bright cyan lines and perfect spacing
    url = "file:///C:/Users/sange/OneDrive/Desktop/Cyber_Resilient_Active_Suspension/final_image_render.html"
    
    # Take a massive 2800x1800 screenshot to ensure perfect resolution of the diagram
    hti.screenshot(url=url, save_as='final_system_design.png', size=(2800, 1800))
    print("SUCCESS: Image generated!")
except Exception as e:
    print(f"FAILED: {e}")

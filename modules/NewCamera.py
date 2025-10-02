"""Module pour le traitement de l'image de la caméra."""
import cv2
import numpy as np

BLUE_LOW = np.array([100, 50, 50])
BLUE_HIGH = np.array([110, 255, 255])

RED_LOW = np.array([0, 50, 50])
RED_HIGH = np.array([20, 255, 255])

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([85, 255, 255])

BROWN_LOW = np.array([0, 0, 0])
BROWN_HIGH = np.array([180, 150, 150]) #to be adjusted

COLORS = [(BLUE_LOW, BLUE_HIGH), (RED_LOW, RED_HIGH), (YELLOW_LOW, YELLOW_HIGH)]

class NewCamera:
    """Gestionnaire de caméra pour le robot."""
    
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.capture = None
        
        
    def setup(self):
        """Initialise la connexion avec la caméra."""
        self.capture = cv2.VideoCapture(self.camera_id)
        
        if not self.capture.isOpened():
            raise RuntimeError(f'Impossible d\'ouvrir la caméra {self.camera_id}')
            
    def _ensure_initialized(self):
        """Vérifie que la caméra est initialisée."""
        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError("La caméra n'est pas initialisée. Appelez setup() d'abord.")
    
    def read_frame(self):
        """
        Capture une image de la caméra.
        
        Returns:
            frame: L'image capturée, ou None si échec
        """
        self._ensure_initialized()
        ret, frame = self.capture.read()
        return frame if ret else None
    
    def get_direction(self):
        """
        Calcule l'erreur normalisée de direction par rapport au centre.
"""
        self._ensure_initialized()
        
        return NewCamera.get_direction(self, )
         
    
    def release(self):
        """Libère les ressources de la caméra."""
        if self.capture is not None:
            self.capture.release()
            cv2.destroyAllWindows()
    
    @staticmethod
    def get_contour_center(contour):
        """
        Calcule le centre d'un contour.
        
        Args:
            contour: Contour OpenCV
            
        Returns:
            tuple: (cx, cy) ou None si le contour est vide
        """
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
        return None

    @staticmethod
    def get_biggest_contour(mask):
        """
        Trouve le plus grand contour dans un masque.
        
        Args:
            mask: Image binaire (masque)
            
        Returns:
            Le contour le plus grand, ou None si aucun contour
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None
        
        return max(contours, key=cv2.contourArea)

    @staticmethod
    def get_line_mask(hsv, color_low, color_high):
        """
        Crée un masque binaire basé sur une plage de couleurs HSV.
        
        Args:
            hsv: Image en espace colorimétrique HSV
            color_low: Tuple (H, S, V) pour la borne inférieure
            color_high: Tuple (H, S, V) pour la borne supérieure
            
        Returns:
            Masque binaire
        """
        return cv2.inRange(hsv, color_low, color_high)

    @staticmethod
    def display(frame, mask):
        """
        Affiche la frame et le masque dans des fenêtres séparées.
        
        Args:
            frame: Image originale
            mask: Masque binaire
        """
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)  # Nécessaire pour rafraîchir les fenêtres

    @staticmethod
    def get_direction(contour, frame_width):
        """
        Calcule l'erreur normalisée de direction par rapport au centre.
        
        Args:
            contour: Contour de la ligne détectée
            frame_width: Largeur de l'image en pixels
            
        Returns:
            float: Erreur normalisée entre -1 et 1, ou None si pas de centre
        """
        if frame_width <= 0:
            raise ValueError("frame_width doit être positif")
            
        center = NewCamera.get_contour_center(contour)
        
        if center is None:
            return None
        
        error = center[0] - frame_width / 2
        error_norm = error / (frame_width / 2)
        
        return error_norm
    
#include "FLmitkRenderWindow.h"
#include "mitkOpenGLRenderer.h"
#include <FL/x.H>
#include <FL/Fl.h>
void FLmitkRenderWindow::InitRenderer()
{ 
   // m_InitNeeded = true;
   // m_ResizeNeeded = true;
        
        if(m_Renderer.IsNull())
	      m_Renderer = mitk::OpenGLRenderer::New();
	  
	  m_Renderer->InitRenderer(this);
	    
//	    this->setAutoBufferSwap( false );
} 
void FLmitkRenderWindow::draw() {
  if(m_InitNeeded)
 {
    m_Renderer->SetWindowId( (void *)fl_xid( this ) );
    m_InitNeeded = false;
 }
  if(m_ResizeNeeded)
    {
      m_ResizeNeeded=false;
      m_Renderer->InitSize(w(),h());
    }
   if(visible())
    {
      make_current(); 
      m_Renderer->Paint();
    }
   }


void FLmitkRenderWindow::resize(int x, int y, int w, int h) {
  std::cout << "resize() called" << std::endl; 
  if (visible()) {
    m_Renderer->Resize(w,h);
  }
  Fl_Gl_Window::resize(x,y,w,h);
} 

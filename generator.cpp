 i n t   n _ s a m p l e s   =   600;
 d o u b l e   o u t ,   w a v e [ n _ s a m p l e s ] ,   t i m e _ f q ,   o s c _ u p d a t e _ t i m e ,   f q , pitch_cv, wave_cv, bank_cv, timer_fq = 72000000; 
 
v o i d   i n i t ( ) { 
     
 } 
 


 v o i d   o s c _ u p d a t e ( ) { 
     i n t   i ; 
 
       w h i l e ( 1 ) { 
           t i m e _ f q   =   1/(timer_fq / (n_samples)); 

      if(out1<8388608){
        out1=0
      }
      else{
        out1 =   w a v e [ i ] -8388608; 
      }


      if(out2>8388608){
        out1=0
      }
      else{
        out1 =   w a v e [ i ] ; 
      }


           i + + ; 
 
         o s D e l a y ( t i m e _ f q ) ;   / / o s c i l l a t o r   f r e e q u e n c y 
       } 
 } 
 
 v o i d   DAC _ u p d a t e ( ) { 
     w h i l e ( 1 ) { 

  LL_DAC_ConvertDualData12RightAligned(DAC1, out1, out2);

     o s D e l a y ( 0 . 0 1 ) ;   / / 9 6 k H z 
     } 
 } 
 
 v o i d   f q _ r e a d ( ) { 
     w h i l e ( 1 ) { 
         f q   =   4 4 0 ; 


    pitch_cv = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_1);
    wave_cv = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_2);
    bank_cv = LL_ADC_INJ_ReadConversionData32(ADC1, LL_ADC_INJ_RANK_3)/536870912;
    
         o s D e l a y ( 1 0 ) ; 
     } 
 } 
